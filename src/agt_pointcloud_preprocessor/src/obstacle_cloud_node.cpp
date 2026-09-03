#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto h1 = std::hash<std::int64_t>{}(key.x);
    const auto h2 = std::hash<std::int64_t>{}(key.y);
    const auto h3 = std::hash<std::int64_t>{}(key.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 2U);
  }
};

double deg2rad(const double value)
{
  return value * M_PI / 180.0;
}

double normalize_angle(double value)
{
  while (value > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

bool valid_xyz(const float x, const float y, const float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

}  // namespace

class ObstacleCloudNode : public rclcpp::Node
{
public:
  ObstacleCloudNode()
  : Node("agt_obstacle_cloud_preprocessor"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/agt/navigation/points_obstacles");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    expected_input_frame_ = declare_parameter<std::string>("expected_input_frame", "lidar_link");

    min_range_ = declare_parameter<double>("range.min_m", 0.5);
    max_range_ = declare_parameter<double>("range.max_m", 120.0);

    self_filter_enabled_ = declare_parameter<bool>("self_filter.enabled", true);
    self_center_ = declare_parameter<std::vector<double>>(
      "self_filter.center_xyz", {0.0, 0.0, 0.20});
    self_size_ = declare_parameter<std::vector<double>>(
      "self_filter.size_xyz", {1.023, 0.778, 0.400});
    self_padding_ = declare_parameter<double>("self_filter.padding_m", 0.05);

    rear_enabled_ = declare_parameter<bool>("rear_sector.enabled", false);
    rear_center_rad_ = deg2rad(declare_parameter<double>("rear_sector.center_deg", 180.0));
    rear_half_width_rad_ = 0.5 * deg2rad(
      declare_parameter<double>("rear_sector.width_deg", 10.0));
    rear_min_range_ = declare_parameter<double>("rear_sector.min_range_m", 0.5);
    rear_max_range_ = declare_parameter<double>("rear_sector.max_range_m", 4.0);

    voxel_enabled_ = declare_parameter<bool>("voxel.enabled", true);
    voxel_leaf_ = declare_parameter<double>("voxel.leaf_size_m", 0.20);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    drop_on_tf_failure_ = declare_parameter<bool>("drop_on_tf_failure", true);

    if (self_center_.size() != 3U || self_size_.size() != 3U) {
      throw std::runtime_error("self_filter.center_xyz and size_xyz must each contain 3 values");
    }
    if (min_range_ < 0.0 || max_range_ <= min_range_) {
      throw std::runtime_error("invalid range filter limits");
    }
    if (voxel_enabled_ && voxel_leaf_ <= 0.0) {
      throw std::runtime_error("voxel.leaf_size_m must be > 0 when voxel filter is enabled");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleCloudNode::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Obstacle cloud branch: %s -> %s. This node is navigation-only and must not replace the FAST-LIO2 time-preserving input.",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  bool inside_self_box(const tf2::Vector3 & p) const
  {
    const double hx = 0.5 * self_size_[0] + self_padding_;
    const double hy = 0.5 * self_size_[1] + self_padding_;
    const double hz = 0.5 * self_size_[2] + self_padding_;
    return std::abs(p.x() - self_center_[0]) <= hx &&
           std::abs(p.y() - self_center_[1]) <= hy &&
           std::abs(p.z() - self_center_[2]) <= hz;
  }

  bool inside_rear_sector(const tf2::Vector3 & p) const
  {
    const double planar_range = std::hypot(p.x(), p.y());
    if (planar_range < rear_min_range_ || planar_range > rear_max_range_) {
      return false;
    }
    const double bearing = std::atan2(p.y(), p.x());
    const double delta = normalize_angle(bearing - rear_center_rad_);
    return std::abs(delta) <= rear_half_width_rad_;
  }

  bool get_base_from_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    tf2::Transform & base_from_cloud)
  {
    if (!expected_input_frame_.empty() && cloud.header.frame_id != expected_input_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Obstacle cloud frame is '%s', expected '%s'.",
        cloud.header.frame_id.c_str(), expected_input_frame_.c_str());
      return false;
    }

    try {
      const auto stamped = tf_buffer_.lookupTransform(
        base_frame_, cloud.header.frame_id, rclcpp::Time(cloud.header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      const auto & tr = stamped.transform.translation;
      const auto & qr = stamped.transform.rotation;
      tf2::Quaternion q(qr.x, qr.y, qr.z, qr.w);
      if (q.length2() < 1e-12) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid zero TF quaternion.");
        return false;
      }
      q.normalize();
      base_from_cloud = tf2::Transform(q, tf2::Vector3(tr.x, tr.y, tr.z));
      return true;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s <- %s unavailable at cloud stamp: %s",
        base_frame_.c_str(), cloud.header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping cloud with empty frame_id.");
      return;
    }

    tf2::Transform base_from_cloud;
    const bool need_tf = self_filter_enabled_ || rear_enabled_;
    const bool tf_ok = !need_tf || get_base_from_cloud(*cloud, base_from_cloud);
    if (need_tf && !tf_ok && drop_on_tf_failure_) {
      return;
    }

    std::vector<std::array<float, 3>> accepted;
    accepted.reserve(static_cast<std::size_t>(cloud->width) * cloud->height / 2U);
    std::unordered_set<VoxelKey, VoxelHash> occupied;
    if (voxel_enabled_) {
      occupied.reserve(static_cast<std::size_t>(cloud->width) * cloud->height / 4U);
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (!valid_xyz(x, y, z)) {
          continue;
        }

        const double range = std::sqrt(
          static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z);
        if (range < min_range_ || range > max_range_) {
          continue;
        }

        if (need_tf && tf_ok) {
          const tf2::Vector3 p_base = base_from_cloud * tf2::Vector3(x, y, z);
          if (self_filter_enabled_ && inside_self_box(p_base)) {
            continue;
          }
          if (rear_enabled_ && inside_rear_sector(p_base)) {
            continue;
          }
        }

        if (voxel_enabled_) {
          const VoxelKey key{
            static_cast<std::int64_t>(std::floor(x / voxel_leaf_)),
            static_cast<std::int64_t>(std::floor(y / voxel_leaf_)),
            static_cast<std::int64_t>(std::floor(z / voxel_leaf_))};
          if (!occupied.insert(key).second) {
            continue;
          }
        }

        accepted.push_back({x, y, z});
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input PointCloud2 must contain float x/y/z fields: %s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud->header;
    output.height = 1U;
    output.width = static_cast<std::uint32_t>(accepted.size());
    output.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(accepted.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    for (const auto & p : accepted) {
      *out_x = p[0];
      *out_y = p[1];
      *out_z = p[2];
      ++out_x;
      ++out_y;
      ++out_z;
    }

    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string base_frame_;
  std::string expected_input_frame_;
  double min_range_{};
  double max_range_{};
  bool self_filter_enabled_{};
  std::vector<double> self_center_;
  std::vector<double> self_size_;
  double self_padding_{};
  bool rear_enabled_{};
  double rear_center_rad_{};
  double rear_half_width_rad_{};
  double rear_min_range_{};
  double rear_max_range_{};
  bool voxel_enabled_{};
  double voxel_leaf_{};
  double tf_timeout_sec_{};
  bool drop_on_tf_failure_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleCloudNode>());
  rclcpp::shutdown();
  return 0;
}

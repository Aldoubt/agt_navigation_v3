#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace
{
std::uint64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time ns_to_stamp(const std::uint64_t ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(ns / 1000000000ULL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000ULL);
  return stamp;
}

bool has_field(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  return std::any_of(msg.fields.begin(), msg.fields.end(), [&](const auto & f) {return f.name == name;});
}
}  // namespace

class LivoxFormatBridge : public rclcpp::Node
{
public:
  LivoxFormatBridge() : Node("livox_format_bridge")
  {
    mode_ = declare_parameter<std::string>("mode", "custom_to_pointcloud2");
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/agt/livox/points");
    lidar_id_ = static_cast<std::uint8_t>(declare_parameter<int>("lidar_id", 0));

    if (mode_ == "custom_to_pointcloud2") {
      pc2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
      custom_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LivoxFormatBridge::on_custom, this, std::placeholders::_1));
    } else if (mode_ == "pointcloud2_to_custom") {
      custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic_, rclcpp::SensorDataQoS());
      pc2_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LivoxFormatBridge::on_pc2, this, std::placeholders::_1));
    } else {
      throw std::runtime_error("mode must be custom_to_pointcloud2 or pointcloud2_to_custom");
    }

    RCLCPP_INFO(get_logger(), "Livox bridge mode=%s %s -> %s",
      mode_.c_str(), input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void on_custom(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    // Preserve the Livox base time exactly in PointCloud2 header.stamp so a bridge round-trip
    // can reconstruct CustomMsg.timebase. The frame_id is preserved from the source header.
    out.header.stamp = ns_to_stamp(msg->timebase);
    out.height = 1;
    out.width = static_cast<std::uint32_t>(msg->points.size());
    out.is_dense = false;
    out.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      8,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "tag", 1, sensor_msgs::msg::PointField::UINT8,
      "line", 1, sensor_msgs::msg::PointField::UINT8,
      "offset_time", 1, sensor_msgs::msg::PointField::UINT32,
      "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64);
    mod.resize(msg->points.size());

    sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z"), intensity(out, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> tag(out, "tag"), line(out, "line");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> offset(out, "offset_time");
    sensor_msgs::PointCloud2Iterator<double> timestamp(out, "timestamp");

    for (const auto & p : msg->points) {
      *x = p.x; *y = p.y; *z = p.z;
      *intensity = static_cast<float>(p.reflectivity);
      *tag = p.tag; *line = p.line; *offset = p.offset_time;
      *timestamp = static_cast<double>(msg->timebase + p.offset_time) * 1e-9;
      ++x; ++y; ++z; ++intensity; ++tag; ++line; ++offset; ++timestamp;
    }
    pc2_pub_->publish(out);
  }

  void on_pc2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    for (const auto * required : {"x", "y", "z"}) {
      if (!has_field(*msg, required)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "PointCloud2 missing required field '%s'", required);
        return;
      }
    }

    livox_ros_driver2::msg::CustomMsg out;
    out.header = msg->header;
    out.timebase = stamp_to_ns(msg->header.stamp);
    out.lidar_id = lidar_id_;
    out.point_num = msg->width * msg->height;
    out.points.resize(out.point_num);

    const bool have_intensity = has_field(*msg, "intensity");
    const bool have_tag = has_field(*msg, "tag");
    const bool have_line = has_field(*msg, "line");
    const bool have_offset = has_field(*msg, "offset_time");
    const bool have_timestamp = has_field(*msg, "timestamp");

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> intensity;
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>> tag, line;
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint32_t>> offset;
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<double>> timestamp;
      if (have_intensity) intensity = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
      if (have_tag) tag = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>>(*msg, "tag");
      if (have_line) line = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>>(*msg, "line");
      if (have_offset) offset = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint32_t>>(*msg, "offset_time");
      if (have_timestamp) timestamp = std::make_unique<sensor_msgs::PointCloud2ConstIterator<double>>(*msg, "timestamp");

      for (std::size_t i = 0; i < out.points.size(); ++i, ++x, ++y, ++z) {
        auto & p = out.points[i];
        p.x = *x; p.y = *y; p.z = *z;
        p.reflectivity = intensity ? static_cast<std::uint8_t>(std::clamp(**intensity, 0.0f, 255.0f)) : 0U;
        p.tag = tag ? **tag : 0U;
        p.line = line ? **line : 0U;

        if (offset) {
          p.offset_time = **offset;
        } else if (timestamp) {
          const double base_sec = static_cast<double>(out.timebase) * 1e-9;
          const double dt_ns = (**timestamp - base_sec) * 1e9;
          p.offset_time = static_cast<std::uint32_t>(std::clamp(
            dt_ns, 0.0, static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
        } else {
          p.offset_time = 0U;
        }

        if (intensity) ++(*intensity);
        if (tag) ++(*tag);
        if (line) ++(*line);
        if (offset) ++(*offset);
        if (timestamp) ++(*timestamp);
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unsupported PointCloud2 field layout: %s", ex.what());
      return;
    }

    custom_pub_->publish(out);
  }

  std::string mode_;
  std::string input_topic_;
  std::string output_topic_;
  std::uint8_t lidar_id_{0};
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxFormatBridge>());
  rclcpp::shutdown();
  return 0;
}

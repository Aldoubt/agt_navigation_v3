#include "agt_local_row_perception/core.hpp"

#include <agt_navigation_interfaces/msg/local_row_state.hpp>
#include <agt_navigation_interfaces/msg/odom_quality.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_set>

namespace agt_local_row_perception
{
namespace
{
double seconds(const builtin_interfaces::msg::Time & time)
{return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;}
builtin_interfaces::msg::Time stamp(double time)
{return rclcpp::Time(static_cast<std::int64_t>(std::llround(time * 1e9)));}
Eigen::Isometry3d pose(const geometry_msgs::msg::Pose & p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6) {
    throw std::invalid_argument("invalid pose quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!result.translation().allFinite()) {throw std::invalid_argument("invalid pose translation");}
  result.linear() = q.normalized().toRotationMatrix();
  return result;
}
Eigen::Isometry3d transform(const geometry_msgs::msg::Transform & p)
{
  geometry_msgs::msg::Pose p0;
  p0.position.x = p.translation.x; p0.position.y = p.translation.y; p0.position.z = p.translation.z;
  p0.orientation = p.rotation;
  return pose(p0);
}
const sensor_msgs::msg::PointField * field(const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  for (const auto & candidate : cloud.fields) {if (candidate.name == name) {return &candidate;}}
  return nullptr;
}
template<typename T> double value(const std::uint8_t * bytes)
{T result; std::memcpy(&result, bytes, sizeof(T)); return static_cast<double>(result);}
std::size_t scalar_size(std::uint8_t datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::INT8: case PointField::UINT8: return 1;
    case PointField::INT16: case PointField::UINT16: return 2;
    case PointField::INT32: case PointField::UINT32: case PointField::FLOAT32: return 4;
    case PointField::FLOAT64: return 8;
    default: throw std::invalid_argument("unsupported PointCloud2 field type");
  }
}
double number(const std::uint8_t * bytes, const sensor_msgs::msg::PointField * f,
  std::size_t point_step, double fallback = 0)
{
  if (!f) {return fallback;}
  if (f->count == 0 || f->offset + scalar_size(f->datatype) > point_step) {
    throw std::invalid_argument("PointCloud2 field exceeds point_step");
  }
  bytes += f->offset;
  using sensor_msgs::msg::PointField;
  switch (f->datatype) {
    case PointField::INT8: return value<std::int8_t>(bytes);
    case PointField::UINT8: return value<std::uint8_t>(bytes);
    case PointField::INT16: return value<std::int16_t>(bytes);
    case PointField::UINT16: return value<std::uint16_t>(bytes);
    case PointField::INT32: return value<std::int32_t>(bytes);
    case PointField::UINT32: return value<std::uint32_t>(bytes);
    case PointField::FLOAT32: return value<float>(bytes);
    case PointField::FLOAT64: return value<double>(bytes);
    default: throw std::invalid_argument("unsupported PointCloud2 field type");
  }
}
geometry_msgs::msg::Point message_point(const Eigen::Vector3d & p)
{geometry_msgs::msg::Point result; result.x = p.x(); result.y = p.y(); result.z = p.z(); return result;}
}  // namespace

class LocalRowNode : public rclcpp::Node
{
public:
  LocalRowNode()
  : Node("local_row_perception"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    tf_broadcaster_(*this)
  {
    const auto config_file = declare_parameter<std::string>("config_file", "");
    const auto geometry_file = declare_parameter<std::string>("geometry_file", "");
    config_ = load_config(config_file);
    geometry_ = load_geometry(geometry_file);
    // Both a verified file and an explicit runtime opt-in are required.
    const bool field_opt_in = declare_parameter<bool>("field_verified", false);
    geometry_.field_verified = geometry_.field_verified && field_opt_in;
    perception_ = std::make_unique<RowPerception>(config_, geometry_);
    history_ = std::make_unique<PoseHistory>(config_.pose_history_sec, config_.pose_max_gap_sec);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    carrier_frame_ = declare_parameter<std::string>("carrier_frame", "local_row_carrier");
    max_input_age_sec_ = declare_parameter<double>("max_input_age_sec", 0.5);
    max_future_sec_ = declare_parameter<double>("max_future_sec", 0.05);
    pending_wait_sec_ = declare_parameter<double>("pending_wait_sec", 0.20);
    input_deskewed_ = declare_parameter<bool>("input_deskewed", false);
    if (!(max_input_age_sec_ > 0) || !(max_future_sec_ >= 0) || !(pending_wait_sec_ > 0)) {
      throw std::invalid_argument("positive age and wait limits required");
    }
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/agt/livox/points");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/agt/odometry/local");
    const auto quality_topic = declare_parameter<std::string>("odom_quality_topic", "/agt/odometry/quality");
    state_pub_ = create_publisher<agt_navigation_interfaces::msg::LocalRowState>("/agt/local_row/state", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/agt/local_row/path", 10);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/agt/local_row/visibility", 2);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/agt/local_row/markers", 2);
    const auto filtered_topic = declare_parameter<std::string>("filtered_cloud_topic", "/agt/livox/points_self_filtered");
    if (cloud_topic.empty() || filtered_topic.empty() || cloud_topic == filtered_topic ||
      odom_frame_.empty() || geometry_.frame_id.empty() || carrier_frame_.empty() ||
      carrier_frame_ == odom_frame_ || carrier_frame_ == geometry_.frame_id)
    {throw std::invalid_argument("distinct input/output topics and diagnostic carrier frame required");}
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic, rclcpp::SensorDataQoS());
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(100),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {on_odom(*msg);});
    quality_sub_ = create_subscription<agt_navigation_interfaces::msg::OdomQuality>(quality_topic, 10,
      [this](const agt_navigation_interfaces::msg::OdomQuality::SharedPtr msg) {on_quality(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(msg);});
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {process_pending();});
    RCLCPP_INFO(get_logger(), "Local row observer; frame=%s field_verified=%s; raw LIO input is unchanged",
      geometry_.frame_id.c_str(), geometry_.field_verified ? "true" : "false");
  }

private:
  struct Pending
  {
    Scan scan;
    std::chrono::steady_clock::time_point receipt;
    sensor_msgs::msg::PointCloud2::SharedPtr source;
  };
  bool time_valid(double time) const
  {
    const double current = now().seconds();
    return time > 0 && std::isfinite(time) && time <= current + max_future_sec_ &&
           time >= current - max_input_age_sec_;
  }
  void check_clock()
  {
    const double current = now().seconds();
    if (last_clock_ > 0 && current + 1e-8 < last_clock_) {
      history_->reset(epoch_); perception_->reset(epoch_); pending_.clear();
      if (have_quality_) {retired_epochs_.insert(epoch_);}
      have_quality_ = false;
      invalid_state(current, "ros_clock_regressed_waiting_new_odom_epoch");
    }
    last_clock_ = current;
  }
  void on_quality(const agt_navigation_interfaces::msg::OdomQuality & msg)
  {
    check_clock();
    if (!time_valid(seconds(msg.header.stamp))) {return;}
    // Epochs are opaque identities; a random epoch after restart can be smaller.
    if (retired_epochs_.count(msg.odom_epoch)) {return;}
    if (have_quality_ && msg.odom_epoch == epoch_ &&
      seconds(msg.header.stamp) < seconds(quality_.header.stamp)) {return;}
    if (!have_quality_ || msg.odom_epoch != epoch_) {
      if (have_quality_) {retired_epochs_.insert(epoch_);}
      epoch_ = msg.odom_epoch;
      history_->reset(epoch_); perception_->reset(epoch_); pending_.clear();
    }
    quality_ = msg; have_quality_ = true;
    quality_receipt_ = std::chrono::steady_clock::now();
  }
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    check_clock();
    const double source_stamp = seconds(msg.header.stamp);
    if (!time_valid(source_stamp) || msg.header.frame_id != odom_frame_ || msg.child_frame_id.empty()) {return;}
    try {
      Eigen::Isometry3d odom_from_body = pose(msg.pose.pose);
      if (msg.child_frame_id != geometry_.frame_id) {
        const auto child_from_body = tf_buffer_.lookupTransform(msg.child_frame_id, geometry_.frame_id,
          rclcpp::Time(msg.header.stamp), rclcpp::Duration::from_seconds(0));
        odom_from_body = odom_from_body * transform(child_from_body.transform);
      }
      const auto previous_pose = history_->at(history_->latest_stamp(), epoch_);
      if (history_->insert({source_stamp, epoch_, odom_from_body})) {
        const auto & v = msg.twist.twist.linear;
        const auto & w = msg.twist.twist.angular;
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
          !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z))
        {history_->reset(epoch_); invalid_state(source_stamp, "odometry_twist_nonfinite"); return;}
        Eigen::Vector3d body_velocity(v.x, v.y, v.z);
        if (msg.child_frame_id != geometry_.frame_id) {
          const auto child_from_body_msg = tf_buffer_.lookupTransform(msg.child_frame_id, geometry_.frame_id,
            rclcpp::Time(msg.header.stamp), rclcpp::Duration::from_seconds(0));
          const auto child_from_body = transform(child_from_body_msg.transform);
          body_velocity = child_from_body.linear().transpose() *
            (body_velocity + Eigen::Vector3d(w.x, w.y, w.z).cross(child_from_body.translation()));
        }
        speed_mps_ = std::copysign(body_velocity.head<2>().norm(), body_velocity.x());
        if (previous_pose && source_stamp - previous_pose->stamp <= config_.pose_max_gap_sec) {
          const Eigen::Vector3d delta_velocity = odom_from_body.linear().transpose() *
            (odom_from_body.translation() - previous_pose->odom_from_body.translation()) /
            (source_stamp - previous_pose->stamp);
          const double magnitude = std::max(std::abs(speed_mps_), delta_velocity.head<2>().norm());
          speed_mps_ = std::copysign(magnitude,
            body_velocity.x() < -1e-3 || delta_velocity.x() < -1e-3 ? -1.0 : 1.0);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Odometry/body transform unavailable: %s", e.what());
    }
  }
  void invalid_state(double source_stamp, const std::string & reason)
  {
    RowState state;
    state.stamp = std::isfinite(source_stamp) && source_stamp > 0 ? source_stamp : std::max(0.0, now().seconds());
    state.odom_epoch = epoch_;
    state.reasons.push_back(reason); publish(state, false);
  }
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
  {
    check_clock();
    const double base_time = seconds(msg->header.stamp);
    if (!time_valid(base_time)) {invalid_state(base_time, "cloud_source_stamp_invalid"); return;}
    if (msg->is_bigendian) {invalid_state(base_time, "big_endian_cloud_unsupported"); return;}
    try {
      const auto * x = field(*msg, "x"); const auto * y = field(*msg, "y"); const auto * z = field(*msg, "z");
      const auto * intensity = field(*msg, "intensity");
      const auto * point_time = field(*msg, "timestamp"); const auto * offset = field(*msg, "offset_time");
      if (!x || !y || !z) {throw std::invalid_argument("cloud missing xyz");}
      if (!point_time && !offset && !input_deskewed_) {
        throw std::invalid_argument("raw cloud missing per-point timestamp/offset_time");
      }
      if (msg->point_step == 0 || msg->row_step < static_cast<std::uint64_t>(msg->width) * msg->point_step ||
        static_cast<std::uint64_t>(msg->row_step) * msg->height > msg->data.size())
      {throw std::invalid_argument("cloud dimensions exceed data");}
      Scan scan; scan.epoch = epoch_; scan.stamp = base_time;
      const auto body_from_sensor = tf_buffer_.lookupTransform(geometry_.frame_id, msg->header.frame_id,
        rclcpp::Time(msg->header.stamp), rclcpp::Duration::from_seconds(0));
      scan.body_from_sensor = transform(body_from_sensor.transform);
      scan.points.reserve(static_cast<std::size_t>(msg->width) * msg->height);
      for (unsigned row = 0; row < msg->height; ++row) {
        for (unsigned column = 0; column < msg->width; ++column) {
          const auto * bytes = msg->data.data() + row * msg->row_step + column * msg->point_step;
          Point p;
          p.xyz = {number(bytes, x, msg->point_step), number(bytes, y, msg->point_step), number(bytes, z, msg->point_step)};
          // Deskewed input is already at header time; preserved offsets must not be applied twice.
          p.stamp = input_deskewed_ ? base_time : point_time ? number(bytes, point_time, msg->point_step) :
            base_time + number(bytes, offset, msg->point_step) * 1e-9;
          p.intensity = static_cast<float>(number(bytes, intensity, msg->point_step));
          if (!std::isfinite(p.stamp) || p.stamp < base_time - max_future_sec_ || p.stamp > base_time + 1.0) {
            throw std::invalid_argument("point timestamp inconsistent with cloud base time");
          }
          scan.stamp = std::max(scan.stamp, p.stamp); scan.points.push_back(p);
        }
      }
      if (scan.stamp > now().seconds() + max_future_sec_) {throw std::invalid_argument("cloud reference is future dated");}
      while (pending_.size() >= 2) {
        invalid_state(pending_.front().scan.stamp, "pending_cloud_queue_overflow");
        pending_.pop_front();
      }
      pending_.push_back({std::move(scan), std::chrono::steady_clock::now(), msg});
      last_cloud_receipt_ = std::chrono::steady_clock::now();
      last_cloud_stamp_ = base_time; cloud_timeout_reported_ = false;
    } catch (const std::exception & e) {invalid_state(base_time, e.what());}
  }
  void publish_filtered(const Scan & scan, const sensor_msgs::msg::PointCloud2 & source)
  {
    auto cloud = source;
    cloud.height = 1; cloud.width = 0; cloud.data.clear();
    cloud.data.reserve(source.data.size());
    std::size_t index = 0;
    const auto origin = scan.body_from_sensor.translation().eval();
    for (unsigned row = 0; row < source.height; ++row) {
      for (unsigned column = 0; column < source.width; ++column, ++index) {
        const auto & p = scan.points[index];
        if (!p.xyz.allFinite()) {continue;}
        const Eigen::Vector3d body = scan.body_from_sensor * p.xyz;
        bool rejected = false;
        for (const auto & shape : geometry_.self_shapes) {
          if (shape.contains(body) || shape.blocks_ray(origin, body)) {rejected = true; break;}
        }
        if (rejected) {continue;}
        const auto * bytes = source.data.data() + row * source.row_step + column * source.point_step;
        cloud.data.insert(cloud.data.end(), bytes, bytes + cloud.point_step); ++cloud.width;
      }
    }
    cloud.row_step = cloud.width * cloud.point_step; cloud.is_dense = false;
    filtered_pub_->publish(cloud); // Every original field, including point timing, is retained.
  }
  void process_pending()
  {
    check_clock();
    const double current = now().seconds();
    const auto steady_now = std::chrono::steady_clock::now();
    if (last_cloud_stamp_ > 0 && !cloud_timeout_reported_ &&
      std::chrono::duration<double>(steady_now - last_cloud_receipt_).count() > max_input_age_sec_) {
      invalid_state(last_cloud_stamp_, "cloud_receipt_watchdog_timeout");
      cloud_timeout_reported_ = true;
    }
    if (pending_.empty()) {return;}
    auto & candidate = pending_.front();
    if (candidate.scan.epoch != epoch_ || current - candidate.scan.stamp > max_input_age_sec_) {
      invalid_state(candidate.scan.stamp, "pending_cloud_stale_or_old_epoch"); pending_.pop_front(); return;
    }
    if (history_->latest_stamp() < candidate.scan.stamp) {
      if (std::chrono::duration<double>(steady_now - candidate.receipt).count() > pending_wait_sec_) {
        invalid_state(candidate.scan.stamp, "point_pose_wait_timeout"); pending_.pop_front();
      }
      return;
    }
    auto item = std::move(candidate); pending_.pop_front();
    // The mounted-body filter does not rewrite coordinates, but its research
    // consumers require a complete acquisition interval. Never replace a failed
    // TF/pose dependency with a superficially valid copy of the original cloud.
    bool complete_interval = history_->at(item.scan.stamp, epoch_).has_value();
    for (const auto & point : item.scan.points) {
      if (point.xyz.allFinite() && !history_->at(point.stamp, epoch_)) {
        complete_interval = false; break;
      }
    }
    if (!complete_interval) {
      invalid_state(item.scan.stamp, "point_pose_interval_incomplete"); return;
    }
    publish_filtered(item.scan, *item.source);
    auto state = perception_->process(item.scan, *history_, speed_mps_);
    const bool odom_quality_valid = have_quality_ && quality_.valid && quality_.odom_epoch == epoch_ &&
      time_valid(seconds(quality_.header.stamp)) && quality_.input_age_sec <= max_input_age_sec_ &&
      quality_.input_age_sec >= 0 && quality_.output_age_sec >= 0 &&
      std::isfinite(quality_.input_age_sec) && std::isfinite(quality_.output_age_sec) &&
      quality_.output_age_sec <= max_input_age_sec_ && std::isfinite(quality_.quality) && quality_.quality > 0 &&
      std::chrono::duration<double>(steady_now - quality_receipt_).count() <= max_input_age_sec_;
    if (!odom_quality_valid) {
      state.row_valid = false; state.clearance_valid = false; state.odom_centerline.clear();
      state.reasons.push_back("odometry_quality_unavailable_or_invalid");
    }
    publish(state, history_->at(item.scan.stamp, epoch_).has_value());
  }
  void publish(const RowState & state, bool have_pose)
  {
    const auto source_stamp = stamp(state.stamp);
    agt_navigation_interfaces::msg::LocalRowState msg;
    msg.header.stamp = source_stamp; msg.header.frame_id = carrier_frame_; msg.odom_epoch = state.odom_epoch;
    msg.ground_valid = state.ground_valid; msg.row_valid = state.row_valid; msg.clearance_valid = state.clearance_valid;
    msg.lateral_error_m = state.lateral_error_m; msg.heading_error_rad = state.heading_error_rad;
    msg.width_m = state.width_m; msg.boundary_left_m = state.boundary_left_m; msg.boundary_right_m = state.boundary_right_m;
    msg.clearance_left_m = state.clearance_left_m; msg.clearance_right_m = state.clearance_right_m;
    msg.clearance_front_m = state.clearance_front_m; msg.observed_forward_m = state.observed_forward_m;
    msg.error_covariance = {state.error_covariance(0, 0), state.error_covariance(0, 1),
      state.error_covariance(1, 0), state.error_covariance(1, 1)};
    msg.quality = state.quality; msg.quality_uncertainty = state.quality_uncertainty;
    msg.quality_support = state.quality_support; msg.quality_temporal = state.quality_temporal;
    msg.quality_width = state.quality_width; msg.left_support = state.left_support;
    msg.right_support = state.right_support; msg.frame_support = state.frame_support; msg.reasons = state.reasons;
    state_pub_->publish(msg);
    if (have_pose) {
      geometry_msgs::msg::TransformStamped carrier;
      carrier.header.stamp = source_stamp; carrier.header.frame_id = odom_frame_; carrier.child_frame_id = carrier_frame_;
      carrier.transform.translation.x = state.odom_from_carrier.translation().x();
      carrier.transform.translation.y = state.odom_from_carrier.translation().y();
      carrier.transform.translation.z = state.odom_from_carrier.translation().z();
      const Eigen::Quaterniond q(state.odom_from_carrier.linear());
      carrier.transform.rotation.x = q.x(); carrier.transform.rotation.y = q.y();
      carrier.transform.rotation.z = q.z(); carrier.transform.rotation.w = q.w();
      tf_broadcaster_.sendTransform(carrier);
    }
    nav_msgs::msg::Path path;
    path.header.stamp = source_stamp; path.header.frame_id = odom_frame_;
    for (std::size_t index = 0; index < state.odom_centerline.size(); ++index) {
      geometry_msgs::msg::PoseStamped p; p.header = path.header;
      p.pose.position = message_point(state.odom_centerline[index]);
      Eigen::Vector3d direction = index + 1 < state.odom_centerline.size() ?
        (state.odom_centerline[index + 1] - state.odom_centerline[index]).eval() :
        (state.odom_from_carrier.linear() * Eigen::Vector3d(1, state.line_parameters.x(), 0)).eval();
      const double heading = std::atan2(direction.y(), direction.x());
      p.pose.orientation.z = std::sin(heading / 2); p.pose.orientation.w = std::cos(heading / 2);
      path.poses.push_back(p);
    }
    path_pub_->publish(path);
    nav_msgs::msg::OccupancyGrid grid;
    grid.header = msg.header; grid.info.map_load_time = source_stamp;
    grid.info.resolution = static_cast<float>(config_.grid_resolution_m);
    grid.info.width = state.grid_width; grid.info.height = state.grid_height;
    grid.info.origin.position.x = config_.x_min_m; grid.info.origin.position.y = -config_.y_abs_max_m;
    grid.info.origin.orientation.w = 1.0; grid.data.reserve(state.cells.size());
    for (const auto & cell : state.cells) {
      grid.data.push_back(cell.classification == CellClass::Free ? 0 :
        (cell.classification == CellClass::Occupied || cell.classification == CellClass::Robot) ? 100 : -1);
    }
    grid_pub_->publish(grid);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker reset;
    reset.header = msg.header; reset.action = visualization_msgs::msg::Marker::DELETEALL; markers.markers.push_back(reset);
    visualization_msgs::msg::Marker coverage;
    coverage.header = msg.header; coverage.ns = "visibility"; coverage.id = 0;
    coverage.type = visualization_msgs::msg::Marker::CUBE_LIST; coverage.action = visualization_msgs::msg::Marker::ADD;
    coverage.pose.orientation.w = 1; coverage.scale.x = config_.grid_resolution_m;
    coverage.scale.y = config_.grid_resolution_m; coverage.scale.z = 0.025;
    coverage.lifetime = rclcpp::Duration::from_seconds(max_input_age_sec_);
    for (const auto & cell : state.cells) {
      coverage.points.push_back(message_point(Eigen::Vector3d(cell.center.x(), cell.center.y(), 0)));
      std_msgs::msg::ColorRGBA color; color.a = 0.35F;
      switch (cell.classification) {
        case CellClass::Free: color.g = 0.9F; break;
        case CellClass::Occupied: color.r = 1.0F; break;
        case CellClass::Robot: color.b = 0.8F; break;
        case CellClass::Occluded: color.r = 1.0F; color.g = 0.5F; break;
        case CellClass::Stale: color.r = color.g = color.b = 0.5F; break;
        default: color.r = color.g = color.b = 0.15F; color.a = 0.1F; break;
      }
      coverage.colors.push_back(color);
    }
    markers.markers.push_back(coverage); marker_pub_->publish(markers);
    diagnostic_msgs::msg::DiagnosticArray diagnostics; diagnostics.header = msg.header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "local_row/observability"; status.hardware_id = geometry_.frame_id;
    status.level = msg.row_valid ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = msg.reasons.empty() ? "row observed" : msg.reasons.front();
    const auto add = [&](const std::string & key, const std::string & v) {
        diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = v; status.values.push_back(kv);
      };
    add("ground_valid", msg.ground_valid ? "true" : "false"); add("row_valid", msg.row_valid ? "true" : "false");
    add("clearance_valid", msg.clearance_valid ? "true" : "false"); add("field_verified", geometry_.field_verified ? "true" : "false");
    add("quality", std::to_string(msg.quality)); add("odom_epoch", std::to_string(epoch_));
    add("points_input", std::to_string(state.points_input)); add("points_self", std::to_string(state.points_self));
    add("points_occluded", std::to_string(state.points_occluded)); add("points_missing_pose", std::to_string(state.points_missing_pose));
    add("independent_frames", std::to_string(state.frame_support)); add("observed_forward_m", std::to_string(state.observed_forward_m));
    for (std::size_t index = 0; index < state.reasons.size(); ++index) {add("reason_" + std::to_string(index), state.reasons[index]);}
    diagnostics.status.push_back(status); diagnostic_pub_->publish(diagnostics);
  }
  Config config_;
  Geometry geometry_;
  std::unique_ptr<RowPerception> perception_;
  std::unique_ptr<PoseHistory> history_;
  std::string odom_frame_, carrier_frame_;
  double max_input_age_sec_, max_future_sec_, pending_wait_sec_, speed_mps_{0};
  bool input_deskewed_{false}, have_quality_{false};
  bool cloud_timeout_reported_{false};
  double last_clock_{0.0}, last_cloud_stamp_{0.0};
  std::chrono::steady_clock::time_point quality_receipt_, last_cloud_receipt_;
  std::unordered_set<std::uint64_t> retired_epochs_;
  std::uint64_t epoch_{0};
  agt_navigation_interfaces::msg::OdomQuality quality_;
  std::deque<Pending> pending_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<agt_navigation_interfaces::msg::OdomQuality>::SharedPtr quality_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<agt_navigation_interfaces::msg::LocalRowState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace agt_local_row_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<agt_local_row_perception::LocalRowNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("local_row_perception"), "%s", e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}

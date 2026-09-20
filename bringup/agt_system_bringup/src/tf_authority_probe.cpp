#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

std::string gid_hex(const uint8_t * data)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < RMW_GID_STORAGE_SIZE; ++index) {
    stream << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return stream.str();
}

template<typename ContainerT>
std::string gid_hex(const ContainerT & data)
{
  return gid_hex(data.data());
}

std::string full_node_name(const std::string & node_namespace, const std::string & node_name)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + node_name;
  }
  return node_namespace + (node_namespace.back() == '/' ? "" : "/") + node_name;
}

class TfAuthorityProbe : public rclcpp::Node
{
public:
  TfAuthorityProbe()
  : Node("agt_tf_authority_probe")
  {
    auto dynamic_qos = rclcpp::QoS(rclcpp::KeepLast(200)).best_effort().durability_volatile();
    auto static_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
    dynamic_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", dynamic_qos,
      [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr message,
      const rclcpp::MessageInfo & info) {record("/tf", message, info);});
    static_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", static_qos,
      [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr message,
      const rclcpp::MessageInfo & info) {record("/tf_static", message, info);});
  }

  void print_evidence()
  {
    for (const auto & topic : {std::string("/tf"), std::string("/tf_static")}) {
      for (const auto & endpoint : get_publishers_info_by_topic(topic)) {
        std::cout << "PUB\t" << topic << '\t' << gid_hex(endpoint.endpoint_gid()) << '\t'
                  << full_node_name(endpoint.node_namespace(), endpoint.node_name()) << '\n';
      }
    }
    for (const auto & entry : observations_) {
      const auto & key = entry.first;
      std::cout << "TF\t" << std::get<0>(key) << '\t' << std::get<1>(key) << '\t'
                << std::get<2>(key) << '\t' << std::get<3>(key) << '\t' << entry.second << '\n';
    }
  }

private:
  using ObservationKey = std::tuple<std::string, std::string, std::string, std::string>;

  void record(
    const std::string & topic,
    const tf2_msgs::msg::TFMessage::ConstSharedPtr & message,
    const rclcpp::MessageInfo & info)
  {
    const auto & rmw_info = info.get_rmw_message_info();
    const std::string gid = gid_hex(rmw_info.publisher_gid.data);
    for (const auto & transform : message->transforms) {
      std::string parent = transform.header.frame_id;
      std::string child = transform.child_frame_id;
      if (!parent.empty() && parent.front() == '/') {
        parent.erase(0, 1);
      }
      if (!child.empty() && child.front() == '/') {
        child.erase(0, 1);
      }
      ++observations_[std::make_tuple(topic, parent, child, gid)];
    }
  }

  std::map<ObservationKey, std::uint64_t> observations_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_subscription_;
};

double parse_duration(int argc, char ** argv)
{
  double duration = 8.0;
  for (int index = 1; index + 1 < argc; ++index) {
    if (std::string(argv[index]) == "--duration-sec") {
      duration = std::stod(argv[index + 1]);
    }
  }
  if (duration <= 0.0) {
    throw std::invalid_argument("duration must be positive");
  }
  return duration;
}

}  // namespace

int main(int argc, char ** argv)
{
  double duration;
  try {
    duration = parse_duration(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "argument error: " << error.what() << '\n';
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfAuthorityProbe>();
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(duration));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  rclcpp::spin_some(node);
  node->print_evidence();
  rclcpp::shutdown();
  return 0;
}

#pragma once
#include <memory>
#include <vector>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <rviz_common/tool.hpp>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

namespace agt_rviz_route_tools {
class RouteTool : public rviz_common::Tool {
public:
  explicit RouteTool(bool freehand, bool straight=false);
  ~RouteTool() override;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;
private:
  void redraw();
  void submit();
  void publishMetrics(bool active, bool force=false);
  void receive(const QString & text);
  bool straight_=false, enabled_=false, continue_straight_=false;
  bool freehand_, dragging_ = false, locked_ = true, pending_ = false;
  int selected_ = -1;
  qint64 revision_ = -1, gesture_revision_ = -1;
  std::vector<Ogre::Vector3> points_, canonical_;
  QElapsedTimer received_, pending_timer_, metrics_sent_;
  QTimer timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_, metrics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<rviz_rendering::BillboardLine> line_;
};
class DrawRouteTool : public RouteTool { public: DrawRouteTool(): RouteTool(true) {} };
class StraightRouteTool : public RouteTool { public: StraightRouteTool(): RouteTool(false,true) {} };
class EditRouteTool : public RouteTool { public: EditRouteTool(): RouteTool(false) {} };

class RoutePanel : public rviz_common::Panel {
public:
  explicit RoutePanel(QWidget * parent = nullptr);
  void onInitialize() override;
private:
  void request(const QString & name, bool confirm);
  QLabel * status_;
  QLabel * reply_;
  QLabel * metrics_;
  QString committed_metrics_;
  bool drawing_metrics_active_=false;
  QElapsedTimer metrics_received_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metrics_sub_;
  QPushButton * execute_;
  QPushButton * resume_;
  QElapsedTimer received_;
  QTimer timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
};
}

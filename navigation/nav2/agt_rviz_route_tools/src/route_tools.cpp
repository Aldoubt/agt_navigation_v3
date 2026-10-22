#include "agt_rviz_route_tools/route_tools.hpp"
#include "agt_rviz_route_tools/chinese.hpp"
#include "agt_rviz_route_tools/route_metrics.hpp"
#include <algorithm>
#include <cmath>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QMessageBox>
#include <QPointer>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QGroupBox>
#include <QFormLayout>
#include <QFrame>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QGridLayout>
#include <OgreCamera.h>
#include <OgreRay.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/render_window.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace agt_rviz_route_tools {
RouteTool::RouteTool(bool freehand, bool straight): straight_(straight), freehand_(freehand) {
  shortcut_key_ = straight ? 'l' : (freehand ? 'd' : 'e');
  received_.invalidate();
  QObject::connect(&timer_, &QTimer::timeout, this, [this]() {
    if (dragging_) publishMetrics(true);
    if (pending_ && pending_timer_.elapsed() > 2000) {
      pending_ = false; continue_straight_=false; points_ = canonical_; redraw();
      setStatus("编辑确认超时，已重新加载草稿，请查看工作台提示。");
    }
  });
  timer_.start(200);
}
RouteTool::~RouteTool() = default;
void RouteTool::onInitialize() {
  setName(straight_ ? "直线绘制" : (freehand_ ? "连续画线" : "航点编辑"));
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  metrics_pub_ = node_->create_publisher<std_msgs::msg::String>("/agt/path_tool/drawing_metrics", 1);
  publisher_ = node_->create_publisher<std_msgs::msg::String>("/agt/path_tool/edit", 10);
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/agt/path_tool/editor_state", rclcpp::QoS(1).transient_local().reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      auto text = QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(this, [this, text]() {receive(text);}, Qt::QueuedConnection);
    });
  line_ = std::make_unique<rviz_rendering::BillboardLine>(context_->getSceneManager());
  line_->setMaxPointsPerLine(2002); line_->setNumLines(1); line_->setLineWidth(0.08f);
  line_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
}
void RouteTool::activate() {
  enabled_=true;
  if(straight_) {
    setStatus("点击起点，移动鼠标查看本段距离，再点击终点；可连续接段，Esc 结束。已有路线从末点续画。");
    return;
  }
  setStatus(freehand_ ? "按住鼠标左键画线，松开提交一笔；Esc 取消当前笔。"
    : "点击添加航点，拖动附近顶点可修改；Delete 删除选中点，Esc 取消当前编辑。");
}
void RouteTool::deactivate() {
  enabled_=false; continue_straight_=false;
  publishMetrics(false, true);
  dragging_ = false; selected_ = -1; points_ = canonical_;
  if (line_) line_->clear();
}
void RouteTool::receive(const QString & text) {
  auto document = QJsonDocument::fromJson(text.toUtf8());
  if (!document.isObject()) return;
  auto object = document.object();
  auto revision = static_cast<qint64>(object["revision"].toDouble(-1));
  received_.restart();
  locked_ = object["locked"].toBool(true);
  canonical_.clear();
  for (auto v : object["points"].toArray()) {
    auto a = v.toArray();
    if (a.size() != 2 || canonical_.size() >= 2000) return;
    canonical_.emplace_back(a[0].toDouble(), a[1].toDouble(), 0.05f);
  }
  const bool resume_chain = straight_ && enabled_ && continue_straight_ && pending_ &&
    !locked_ && revision>gesture_revision_ && canonical_==points_ && canonical_.size()<2000;
  if (locked_ || revision != revision_) {
    continue_straight_=false;
    dragging_ = false; if (!pending_ || locked_) selected_ = -1; pending_ = false;
    points_ = canonical_;
    if (line_) line_->clear();
  }
  revision_ = revision;
  if(resume_chain) {
    points_=canonical_;points_.push_back(points_.back());
    dragging_=true;gesture_revision_=revision_;redraw();publishMetrics(true,true);
  }
  if (pending_ && points_ == canonical_) pending_ = false;
  // Rejected edits are acknowledged by status without advancing revision.
  if (pending_ && object["status"].toString().startsWith("EDIT REJECTED")) {
    pending_ = false; continue_straight_=false; points_ = canonical_; if (line_) line_->clear();
  }
}
void RouteTool::redraw() {
  if (!line_) return;
  line_->clear();
  if (points_.size() == 1) {
    auto p = points_.front();
    line_->addPoint(p-Ogre::Vector3(0.04f,0,0));
    line_->addPoint(p+Ogre::Vector3(0.04f,0,0));
  } else for (auto & p : points_) line_->addPoint(p);
  if(straight_ && dragging_ && points_.size()>=2 && points_.back()==points_[points_.size()-2]) {
    // Show the anchor immediately, even before the cursor has moved.
    auto p=points_.back();line_->addPoint(p-Ogre::Vector3(.1f,0,0));line_->addPoint(p+Ogre::Vector3(.1f,0,0));
  }
  context_->queueRender();
}
void RouteTool::publishMetrics(bool active, bool force) {
  if(!metrics_pub_ || (!force && metrics_sent_.isValid() && metrics_sent_.elapsed()<100))return;
  metrics_sent_.restart();
  QJsonArray points;
  for(const auto & p:points_)points.append(QJsonArray{p.x,p.y});
  auto m=measureRoute(points);
  QJsonObject object{{"active",active},{"total",m.total},{"last",m.last},{"count",m.count},{"straight",straight_}};
  std_msgs::msg::String message;
  message.data=QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
  metrics_pub_->publish(message);
}
void RouteTool::submit() {
  publishMetrics(false, true);
  QJsonArray points;
  for (auto & p : points_) points.append(QJsonArray{p.x,p.y});
  QJsonObject object{{"frame", "map"}, {"revision", static_cast<double>(gesture_revision_)}, {"points", points}};
  std_msgs::msg::String message;
  message.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
  publisher_->publish(message);
  pending_ = true; pending_timer_.restart();
}
int RouteTool::processMouseEvent(rviz_common::ViewportMouseEvent & event) {
  // Tools must forward camera gestures, even while the draft/backend is locked.
  if(event.wheel_delta!=0 || ((!dragging_ || straight_) && (event.middle() || event.middleDown() || event.middleUp()))) {
    if(event.panel->getViewController())event.panel->getViewController()->handleMouseEvent(event);
    return Render;
  }
  if (context_->getFixedFrame() != "map" || locked_ || !received_.isValid() || received_.elapsed()>2000 || pending_) {
    if (dragging_) {dragging_=false; points_=canonical_; if(line_)line_->clear();}
    setStatus("暂时无法绘制：需要 map 坐标系、有效后端连接且草稿未锁定。");
    return Render;
  }
  auto * render_window=event.panel->getRenderWindow();
  auto * camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window);
  // ViewportMouseEvent contains device-scaled window coordinates, not QWidget panel coordinates.
  const int pixel_ratio=std::max(1,event.device_pixel_ratio);
  const float viewport_width=render_window->width()*pixel_ratio;
  const float viewport_height=render_window->height()*pixel_ratio;
  if (!camera || viewport_width<=0 || viewport_height<=0) return 0;
  auto ray = camera->getCameraToViewportRay(static_cast<float>(event.x)/viewport_width,
                                          static_cast<float>(event.y)/viewport_height);
  auto hit = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, 0));
  if (!hit.first || hit.second<0) return 0;
  auto point = ray.getPoint(hit.second); point.z = 0.05f;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x)>100000 || std::abs(point.y)>100000) return 0;
  auto width_ray = camera->getCameraToViewportRay(static_cast<float>(event.x+pixel_ratio)/viewport_width,
                                                static_cast<float>(event.y)/viewport_height);
  auto width_hit = width_ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z,0));
  if(width_hit.first)line_->setLineWidth(std::max(0.003f,(width_ray.getPoint(width_hit.second)-Ogre::Vector3(point.x,point.y,0)).length()));
  if(straight_) {
    if(event.rightDown()) {
      continue_straight_=false;dragging_=false;points_=canonical_;line_->clear();publishMetrics(false,true);
      return Render;
    }
    if(event.leftDown()) {
      event.panel->setFocus(Qt::MouseFocusReason);
      if(!dragging_) {
        if(canonical_.size()>=2000){setStatus("最多 2000 个顶点，请先撤销或清空。");return Render;}
        points_=canonical_;gesture_revision_=revision_;selected_=-1;
        if(points_.empty())points_.push_back(point);
        points_.push_back(point);dragging_=true;redraw();publishMetrics(true,true);
      } else {
        points_.back()=point;
        if((point-points_[points_.size()-2]).length()<0.01f) {
          setStatus("终点离起点过近，请移动鼠标后再点击。");redraw();return Render;
        }
        dragging_=false;continue_straight_=true;redraw();submit();
      }
    } else if(dragging_ && event.type==QEvent::MouseMove) {
      points_.back()=point;redraw();publishMetrics(true);
      setStatus("直线预览：单击确认终点并继续下一段；Esc 或右键结束。");
    }
    return Render;
  }
  if (event.leftDown()) {
    event.panel->setFocus(Qt::MouseFocusReason);
    dragging_=true; gesture_revision_=revision_; points_=canonical_; selected_=-1;
    if (!freehand_) {
      auto pick_ray = camera->getCameraToViewportRay(static_cast<float>(event.x+10*pixel_ratio)/viewport_width,
                                                    static_cast<float>(event.y)/viewport_height);
      auto pick_hit = pick_ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, 0));
      float distance = pick_hit.first ? (pick_ray.getPoint(pick_hit.second)-Ogre::Vector3(point.x,point.y,0)).length() : 0.25f;
      for (size_t i=0;i<points_.size();++i) {
        auto d = (points_[i]-point).length();
        if (d<distance) {distance=d;selected_=static_cast<int>(i);}
      }
    }
    if (selected_<0) {
      if (points_.size()>=2000) {dragging_=false;setStatus("最多 2000 个顶点，请撤销操作或清空草稿。");return Render;}
      points_.push_back(point); selected_=static_cast<int>(points_.size()-1);
    }
    redraw();
  } else if (dragging_ && (event.left() || event.leftUp())) {
    if (freehand_) {
      if ((point-points_.back()).length()>=0.03f && points_.size()<2000) {
        points_.push_back(point);
        // Incremental line update: no ROS round trip and no full route resampling per mouse move.
        if (points_.size()<=2) redraw(); else {line_->addPoint(point);context_->queueRender();}
      }
    } else if (selected_>=0) {points_[selected_]=point;redraw();}
  }
  if (dragging_) publishMetrics(true);
  if (event.leftUp() && dragging_) {dragging_=false;submit();}
  if (event.rightDown()) {publishMetrics(false,true);dragging_=false;points_=canonical_;if(line_)line_->clear();}
  return Render;
}
int RouteTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel *) {
  if (event->key()==Qt::Key_Escape) {
    continue_straight_=false;
    publishMetrics(false,true);dragging_=false;points_=canonical_;if(line_)line_->clear();return Render;
  }
  if (event->key()==Qt::Key_Delete && !straight_ && !freehand_ && !locked_ && !pending_ &&
      received_.isValid() && received_.elapsed()<2000 && selected_>=0 &&
      static_cast<size_t>(selected_)<points_.size()) {
    gesture_revision_=revision_;points_.erase(points_.begin()+selected_);selected_=-1;
    redraw();submit();return Render;
  }
  return 0;
}

RoutePanel::RoutePanel(QWidget * parent): rviz_common::Panel(parent) {
  auto * outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(2, 2, 2, 2);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * content = new QWidget(scroll);
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(4, 4, 4, 4);
  scroll->setWidget(content);
  outer_layout->addWidget(scroll);
  auto * instructions = new QLabel("默认从机器人当前位姿生成直线路线：输入距离即可生成草稿。地图视图支持滚轮缩放、中键平移；居中按钮只移动视图，不控制车辆。生成后先预览／校验，再单独确认执行。手绘工具仍可选用。");
  instructions->setWordWrap(true); layout->addWidget(instructions);
  auto * generator = new QGroupBox("参数化直线路线");
  auto * generator_layout = new QFormLayout(generator);
  distance_input_ = new QDoubleSpinBox(generator);
  distance_input_->setObjectName("route_distance_input");
  distance_input_->setRange(0.1, 200.0);
  distance_input_->setDecimals(2);
  distance_input_->setSingleStep(0.5);
  distance_input_->setValue(5.0);
  distance_input_->setSuffix(" m");
  heading_mode_ = new QComboBox(generator);
  heading_mode_->setObjectName("route_heading_mode");
  heading_mode_->addItem("沿机器人当前航向");
  heading_mode_->addItem("指定地图航向");
  heading_input_ = new QDoubleSpinBox(generator);
  heading_input_->setObjectName("route_heading_input");
  heading_input_->setRange(-180.0, 180.0);
  heading_input_->setDecimals(1);
  heading_input_->setSingleStep(5.0);
  heading_input_->setValue(0.0);
  heading_input_->setSuffix(" °");
  heading_input_->setToolTip("map 坐标系航向：0° 沿 +X，正角度逆时针；如果车头尚未对齐，在线预览可能要求先调整车身或规划连接段。");
  heading_input_->setEnabled(false);
  generator_layout->addRow("行驶距离", distance_input_);
  generator_layout->addRow("行驶方向", heading_mode_);
  generator_layout->addRow("地图航向", heading_input_);
  generate_route_ = new QPushButton("按距离生成路线草稿", generator);
  generate_route_->setObjectName("generate_straight_route_button");
  generate_route_->setToolTip("从 map → base_link 当前位姿生成直线；只更新草稿，不会发送运动命令。");
  generator_layout->addRow(generate_route_);
  layout->addWidget(generator);
  QObject::connect(heading_mode_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    [this](int index) { heading_input_->setEnabled(index == 1); });
  status_=new QLabel("正在等待工作台后端连接……");status_->setWordWrap(true);
  status_->setTextFormat(Qt::PlainText);layout->addWidget(status_);
  metrics_=new QLabel("草稿总长：0.00 米｜点数：0");
  metrics_->setObjectName("route_distance_metrics");
  metrics_->setFixedHeight(78); // Stable viewport geometry while switching live/preview distance text.
  metrics_->setTextFormat(Qt::PlainText);metrics_->setWordWrap(true);
  metrics_->setStyleSheet("QLabel { background: #20242a; color: #fff36a; padding: 7px; font-weight: bold; }");
  layout->addWidget(metrics_);
  auto * buttons = new QGridLayout(); layout->addLayout(buttons);
  center_robot_ = new QPushButton("居中机器人");
  center_robot_->setObjectName("center_robot_button");
  center_robot_->setToolTip("将地图视图中心移动到 map → base_link 的当前机器人位置；保留缩放，不发送运动命令。");
  buttons->addWidget(center_robot_, 0, 0, 1, 2);
  const std::vector<std::pair<QString,QString>> specs = {
    {"预览／校验","preview"},{"执行…","start"},{"撤销","undo"},{"重做","redo"},
    {"清空草稿","clear_route"},{"清空轨迹","clear_trails"},{"暂停","pause"},
    {"恢复…","resume"},{"取消任务","cancel"}};
  int i=2;
  for (const auto & spec: specs) {
    auto * button = new QPushButton(spec.first); buttons->addWidget(button,i/2,i%2);++i;
    if(spec.second=="start") execute_=button;
    if(spec.second=="resume") resume_=button;
    QObject::connect(button,&QPushButton::clicked,this,[this,spec](){request(spec.second,spec.second=="start"||spec.second=="resume");});
  }
  execute_->setEnabled(false);resume_->setEnabled(false);
  center_robot_->setEnabled(false);
  QObject::connect(center_robot_, &QPushButton::clicked, this, [this]() { centerRobot(); });
  QObject::connect(generate_route_, &QPushButton::clicked, this, [this]() { generateStraightRoute(); });
  generate_route_->setEnabled(false);
  reply_=new QLabel();reply_->setWordWrap(true);reply_->setTextFormat(Qt::PlainText);layout->addWidget(reply_);
  QObject::connect(&timer_,&QTimer::timeout,this,[this](){
    if(drawing_metrics_active_ && metrics_received_.isValid() && metrics_received_.elapsed()>800) {
      drawing_metrics_active_=false;metrics_->setText(committed_metrics_);
    }
    if(!received_.isValid()||received_.elapsed()>2000){
      execute_->setEnabled(false);resume_->setEnabled(false);
      status_->setText("后端断开或状态已过期，已禁用执行。请独立确认车辆已停车。");
      metrics_->setText("后端未连接：距离数据不可用");
    }
  });timer_.start(500);
}
void RoutePanel::onInitialize() {
  node_=getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // RViz already spins its ROS node; do not try to add it to a second executor.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
  edit_pub_ = node_->create_publisher<std_msgs::msg::String>("/agt/path_tool/edit", 10);
  center_robot_->setEnabled(true);
  metrics_sub_=node_->create_subscription<std_msgs::msg::String>("/agt/path_tool/drawing_metrics",1,
    [this](std_msgs::msg::String::ConstSharedPtr message){
      auto text=QString::fromStdString(message->data);
      QMetaObject::invokeMethod(this,[this,text](){
        auto o=QJsonDocument::fromJson(text.toUtf8()).object();
        drawing_metrics_active_=o["active"].toBool();metrics_received_.restart();
        RouteMetrics m;m.total=o["total"].toDouble();m.last=o["last"].toDouble();m.count=o["count"].toInt();
        if(!std::isfinite(m.total)||!std::isfinite(m.last))return;
        QString current=metricsText(m,true);
        if(o["straight"].toBool())current=QString("直线预览 · 本段：%1 米\n路线总长：%2 米｜预览点数：%3")
          .arg(m.last,0,'f',2).arg(m.total,0,'f',2).arg(m.count);
        metrics_->setText(drawing_metrics_active_?current:committed_metrics_);
      },Qt::QueuedConnection);
    });
  subscription_=node_->create_subscription<std_msgs::msg::String>("/agt/path_tool/editor_state",
    rclcpp::QoS(1).transient_local().reliable(),[this](std_msgs::msg::String::ConstSharedPtr msg){
      auto text=QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(this,[this,text](){
        auto doc=QJsonDocument::fromJson(text.toUtf8());if(!doc.isObject())return;auto o=doc.object();
        received_.restart();
        draft_revision_ = static_cast<qint64>(o["revision"].toDouble(-1));
        draft_point_count_ = o["points"].toArray().size();
        draft_locked_ = o["locked"].toBool(true);
        generate_route_->setEnabled(draft_revision_ >= 0 && !draft_locked_);
        committed_metrics_=metricsText(measureRoute(o["points"].toArray()),false);
        if(o["preview_length_m"].isDouble())
          committed_metrics_+=QString("\n预览路线总长（实际下发）：%1 米").arg(o["preview_length_m"].toDouble(),0,'f',2);
        if(!drawing_metrics_active_)metrics_->setText(committed_metrics_);
        status_->setText(QString("%1｜草稿版本 %2\n地图：%3 / %4\n导航状态：%5｜已定位：%6\n%7")
          .arg(o["preview_only"].toBool()?"离线模式／禁止运动":"在线模式／可控制车辆运动")
          .arg(o["revision"].toInt()).arg(o["map_id"].toString()).arg(o["map_version"].toString())
          .arg(chinese(o["health"].toString())).arg(o["localized"].toBool()?"是":"否").arg(chinese(o["status"].toString())));
        execute_->setEnabled(o["can_execute"].toBool() && !o["resume_preview"].toBool());
        resume_->setEnabled(o["can_execute"].toBool() && o["resume_preview"].toBool());
      },Qt::QueuedConnection);
    });
}
bool RoutePanel::lookupRobotTransform(geometry_msgs::msg::TransformStamped & transform, QString & error) {
  if (!tf_buffer_) {
    error = "TF 缓冲区尚未初始化";
    return false;
  }
  try {
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException & exception) {
    error = QString::fromUtf8(exception.what());
    return false;
  }
}

void RoutePanel::centerRobot() {
  geometry_msgs::msg::TransformStamped robot_tf;
  QString error;
  if (!lookupRobotTransform(robot_tf, error)) {
    reply_->setText(QString("无法居中：找不到 map → base_link TF（%1）。请先确认定位与 TF 发布。")
      .arg(error));
    return;
  }
  rviz_common::VisualizationFrame * frame = nullptr;
  for (QWidget * widget = this; widget; widget = widget->parentWidget()) {
    frame = dynamic_cast<rviz_common::VisualizationFrame *>(widget);
    if (frame) break;
  }
  auto * manager = frame ? frame->getManager() : nullptr;
  auto * render_panel = manager ? manager->getRenderPanel() : nullptr;
  auto * view = render_panel ? render_panel->getViewController() : nullptr;
  if (!view) {
    reply_->setText("无法居中：没有找到 RViz 当前视图控制器。");
    return;
  }
  const auto & t = robot_tf.transform.translation;
  view->lookAt(Ogre::Vector3(static_cast<float>(t.x), static_cast<float>(t.y), static_cast<float>(t.z)));
  reply_->setText(QString("视图已居中到机器人位置（map：x=%1，y=%2）。当前缩放保持不变；未发送运动命令。")
    .arg(t.x, 0, 'f', 2).arg(t.y, 0, 'f', 2));
}

void RoutePanel::generateStraightRoute() {
  if (!received_.isValid() || received_.elapsed() > 2000 || draft_revision_ < 0) {
    reply_->setText("暂时无法生成：路线后端状态未连接或已过期。");
    return;
  }
  if (draft_locked_) {
    reply_->setText("当前路线已锁定：请先取消正在运行或暂停的任务，再编辑草稿。");
    return;
  }
  if (draft_point_count_ > 0) {
    QMessageBox dialog(QMessageBox::Question, "替换当前路线草稿",
      "生成直线路线将替换当前草稿。是否继续？", QMessageBox::Yes | QMessageBox::No, this);
    dialog.button(QMessageBox::Yes)->setText("替换草稿");
    dialog.button(QMessageBox::No)->setText("保留现有路线");
    dialog.setDefaultButton(QMessageBox::No);
    if (dialog.exec() != QMessageBox::Yes) return;
  }
  geometry_msgs::msg::TransformStamped robot_tf;
  QString error;
  if (!lookupRobotTransform(robot_tf, error)) {
    reply_->setText(QString("无法生成路线：找不到 map → base_link TF（%1）。").arg(error));
    return;
  }
  const auto & t = robot_tf.transform.translation;
  const double distance = distance_input_->value();
  const auto & q = robot_tf.transform.rotation;
  const double robot_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double yaw = heading_mode_->currentIndex() == 0
    ? robot_yaw : heading_input_->value() * 3.14159265358979323846 / 180.0;
  const double end_x = t.x + distance * std::cos(yaw);
  const double end_y = t.y + distance * std::sin(yaw);
  if (!std::isfinite(end_x) || !std::isfinite(end_y)) {
    reply_->setText("无法生成路线：终点坐标不是有效数值。");
    return;
  }
  QJsonArray points;
  points.append(QJsonArray{t.x, t.y});
  points.append(QJsonArray{end_x, end_y});
  QJsonObject payload{{"frame", "map"}, {"revision", static_cast<double>(draft_revision_)}, {"points", points}};
  std_msgs::msg::String message;
  message.data = QJsonDocument(payload).toJson(QJsonDocument::Compact).toStdString();
  if (!edit_pub_) {
    reply_->setText("无法生成路线：编辑接口未初始化。");
    return;
  }
  edit_pub_->publish(message);
  reply_->setText(QString("已提交 %1 米直线路线草稿（起点为机器人当前位置）。请检查地图预览，再点击“预览／校验”；当前没有发送运动命令。")
    .arg(distance, 0, 'f', 2));
}

void RoutePanel::request(const QString & name,bool confirm) {
  if(!node_){reply_->setText("ROS 节点不可用");return;}
  if(confirm && (!received_.isValid()||received_.elapsed()>2000))return;
  if(confirm) {
    QMessageBox dialog(QMessageBox::Warning, "确认执行路线",
      "将向导航能力层提交已校验的路线，车辆可能开始运动。\n"
      "请确认定位与显示路线正确、区域内无障碍，并且急停和遥控器可用。",
      QMessageBox::Yes|QMessageBox::No, this);
    dialog.button(QMessageBox::Yes)->setText("确认执行");
    dialog.button(QMessageBox::No)->setText("返回检查");
    dialog.setDefaultButton(QMessageBox::No);
    if(dialog.exec()!=QMessageBox::Yes)return;
  }
  auto key=name.toStdString();auto & client=clients_[key];
  if(!client)client=node_->create_client<std_srvs::srv::Trigger>("/agt/path_tool/"+key);
  if(!client->service_is_ready()){reply_->setText("服务不可用："+actionChinese(name));return;}
  QPointer<RoutePanel> safe(this);
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [safe](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
      QString result;
      try {auto response=future.get();result=(response->success?"成功：":"已拒绝：")+chinese(QString::fromStdString(response->message));}
      catch(const std::exception & e){result=QString("服务异常：")+e.what();}
      if(safe) QMetaObject::invokeMethod(safe,[safe,result](){if(safe)safe->reply_->setText(result);},Qt::QueuedConnection);
    });
  reply_->setText("已请求："+actionChinese(name));
}
}
PLUGINLIB_EXPORT_CLASS(agt_rviz_route_tools::StraightRouteTool,rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(agt_rviz_route_tools::DrawRouteTool,rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(agt_rviz_route_tools::EditRouteTool,rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(agt_rviz_route_tools::RoutePanel,rviz_common::Panel)

// Isolated integration probe: real RViz/Ogre/Qt events + ROS backend, no motion.
#include <QApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTest>
#include <QPushButton>
#include <QLabel>
#include <QElapsedTimer>
#include <QScreen>
#include <QWheelEvent>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/properties/property.hpp>
#include <iostream>
#include <stdexcept>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc,char **argv) {
  QApplication app(argc,argv);
  rviz_common::ros_integration::RosClientAbstraction ros;
  auto weak=ros.init(argc,argv,"workbench_gui_probe",false);
  auto node=weak.lock()->get_raw_node();
  QJsonObject state;
  auto sub=node->create_subscription<std_msgs::msg::String>("/agt/path_tool/editor_state",
    rclcpp::QoS(1).reliable().transient_local(),[&](std_msgs::msg::String::ConstSharedPtr msg){
      state=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
    });
  int result=0;
  {
    rviz_common::VisualizationFrame frame(weak);
    frame.setApp(&app);frame.setSplashPath("");frame.initialize(weak,argv[1]);frame.show();
    auto wait=[&](auto predicate,int timeout=6000){
      QElapsedTimer timer;timer.start();while(!predicate()&&timer.elapsed()<timeout)QTest::qWait(20);
      if(!predicate())throw std::runtime_error("GUI condition timed out; state="+QJsonDocument(state).toJson().toStdString());
    };
    try {
      wait([&](){return !state.isEmpty() && state["map_received"].toBool();});
      if(!state["preview_only"].toBool())throw std::runtime_error("requires offline backend");
      auto * manager=frame.getManager();auto * tools=manager->getToolManager();
      rviz_common::Tool *draw=nullptr,*edit=nullptr,*straight=nullptr;
      for(int i=0;i<tools->numTools();++i){auto * t=tools->getTool(i);
        if(t->getClassId()=="agt_rviz_route_tools/StraightRoute")straight=t;
        if(t->getClassId()=="agt_rviz_route_tools/DrawRoute")draw=t;
        if(t->getClassId()=="agt_rviz_route_tools/EditWaypoints")edit=t;}
      if(!draw||!edit||!straight)throw std::runtime_error("native tools missing");
      auto findButton=[&](QString name){
        for(auto * b:frame.findChildren<QPushButton*>())if(b->text()==name)return b;
        throw std::runtime_error("button missing: "+name.toStdString());
      };
      auto clear=[&](){QTest::mouseClick(findButton("清空草稿"),Qt::LeftButton);wait([&](){return state["points"].toArray().isEmpty();});QTest::qWait(200);};
      clear();tools->setCurrentTool(draw);QTest::qWait(100);
      auto * window=manager->getRenderPanel()->getRenderWindow();
      QPoint center(window->width()/2,window->height()/2);
      auto zoom=[&](){
        auto * scale=manager->getRenderPanel()->getViewController()->subProp("Scale");
        double original=scale->getValue().toDouble();
        QWheelEvent wheel(QPointF(center),QPointF(window->mapToGlobal(center)),QPoint(),QPoint(0,120),
                          Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
        QApplication::sendEvent(window,&wheel);
        QTest::qWait(150);
        std::cout<<"ZOOM "<<original<<" -> "<<scale->getValue().toDouble()<<std::endl;
        wait([&](){return std::abs(scale->getValue().toDouble()-original)>0.001;});
        double enlarged=scale->getValue().toDouble();
        QWheelEvent back(QPointF(center),QPointF(window->mapToGlobal(center)),QPoint(),QPoint(0,-120),
                          Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
        QApplication::sendEvent(window,&back);
        wait([&](){return std::abs(scale->getValue().toDouble()-enlarged)>0.001;});
        if((enlarged-original)*(scale->getValue().toDouble()-enlarged)>=0)
          throw std::runtime_error("reverse wheel failed to reverse zoom direction");
        scale->setValue(original);QTest::qWait(100);
      };
      zoom();
      auto * view=manager->getRenderPanel()->getViewController();
      double original_x=view->subProp("X")->getValue().toDouble();
      double original_y=view->subProp("Y")->getValue().toDouble();
      QTest::mousePress(window,Qt::MiddleButton,Qt::NoModifier,center);
      QTest::mouseMove(window,center+QPoint(20,10),40);
      QTest::mouseRelease(window,Qt::MiddleButton,Qt::NoModifier,center+QPoint(20,10));
      wait([&](){return std::abs(view->subProp("X")->getValue().toDouble()-original_x)>0.001 ||
                      std::abs(view->subProp("Y")->getValue().toDouble()-original_y)>0.001;});
      if(!state["points"].toArray().isEmpty())throw std::runtime_error("camera gesture edited route");
      view->subProp("X")->setValue(original_x);view->subProp("Y")->setValue(original_y);QTest::qWait(100);
      auto * distance=frame.findChild<QLabel*>("route_distance_metrics");
      if(!distance)throw std::runtime_error("distance label missing");
      // Qt sends actual window mouse input. RViz forwards it into the active tool.
      QTest::mousePress(window,Qt::LeftButton,Qt::NoModifier,center);
      for(int i=1;i<=25;++i)QTest::mouseMove(window,center+QPoint(i*3,i),8);
      wait([&](){return distance->text().contains("正在绘制") && distance->text().contains("米");});
      app.primaryScreen()->grabWindow(frame.winId()).save(QString(argv[2])+"-live-stroke.png");
      QTest::mouseRelease(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(75,25));
      wait([&](){return state["points"].toArray().size()>5;});
      int drawn=state["points"].toArray().size();
      QTest::mouseClick(findButton("预览／校验"),Qt::LeftButton);
      wait([&](){return state["status"].toString().startsWith("OFFLINE GEOMETRY PREVIEW");});
      QTest::qWait(200);
      app.primaryScreen()->grabWindow(frame.winId()).save(QString(argv[2])+"-drawing.png");
      bool chinese_status=false;
      for(auto * label:frame.findChildren<QLabel*>())
        if(label->text().contains("离线几何预览"))chinese_status=true;
      if(!chinese_status)throw std::runtime_error("Chinese preview status missing");
      if(!distance->text().contains("预览总长（含连接段）"))throw std::runtime_error("preview distance missing");
      std::cout<<"DISTANCE "<<distance->text().toStdString()<<std::endl;
      int revision=state["revision"].toInt();
      QTest::mousePress(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(90,0));
      QTest::mouseMove(window,center+QPoint(100,10),30);
      QTest::keyClick(manager->getRenderPanel(),Qt::Key_Escape);
      QTest::mouseRelease(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(100,10));
      QTest::qWait(300);
      if(state["revision"].toInt()!=revision)throw std::runtime_error("Escape committed an unwanted stroke");
      std::cout<<"DRAG_POINTS "<<drawn<<std::endl;
      QTest::mouseClick(findButton("撤销"),Qt::LeftButton);
      wait([&](){return state["points"].toArray().isEmpty();});
      QTest::mouseClick(findButton("重做"),Qt::LeftButton);
      wait([&](){return state["points"].toArray().size()==drawn;});
      clear();tools->setCurrentTool(edit);QTest::qWait(100);zoom();
      QTest::mouseClick(window,Qt::LeftButton,Qt::NoModifier,center);
      wait([&](){return state["points"].toArray().size()==1;});QTest::qWait(150);
      QTest::mouseClick(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(70,0));
      wait([&](){return state["points"].toArray().size()==2;});QTest::qWait(150);
      auto before=state["points"].toArray();
      QTest::mousePress(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(70,0));
      QTest::mouseMove(window,center+QPoint(70,35),100);
      QTest::mouseRelease(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(70,35));
      wait([&](){return state["points"].toArray()!=before;});
      if(state["points"].toArray().size()!=2)throw std::runtime_error("drag appended instead of moving vertex");
      QTest::qWait(200);
      QTest::keyClick(manager->getRenderPanel(),Qt::Key_Delete);
      wait([&](){return state["points"].toArray().size()==1;});
      clear();tools->setCurrentTool(straight);QTest::qWait(100);
      center=QPoint(window->width()/2,window->height()/2);zoom();
      int first_revision=state["revision"].toInt();
      QTest::mouseClick(window,Qt::LeftButton,Qt::NoModifier,center);
      QTest::mouseClick(window,Qt::LeftButton,Qt::NoModifier,center);
      QTest::qWait(100);
      if(state["revision"].toInt()!=first_revision)throw std::runtime_error("zero-length straight segment was committed");
      // Deliberately wiggle the mouse: only the two clicked endpoints may be committed.
      for(int i=1;i<=20;++i)QTest::mouseMove(window,center+QPoint(i*4,(i%2?15:-15)),10);
      QTest::mouseMove(window,center+QPoint(80,0),30);
      wait([&](){return distance->text().contains("直线预览") && !distance->text().contains("本段：0.00");});
      if(!state["points"].toArray().isEmpty())throw std::runtime_error("hover preview committed points");
      QTest::qWait(250);
      std::cout<<"STRAIGHT_PREVIEW "<<distance->text().toStdString()<<std::endl;
      app.primaryScreen()->grabWindow(frame.winId()).save(QString(argv[2])+"-straight-preview.png");
      QTest::mouseClick(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(80,0));
      wait([&](){return state["points"].toArray().size()==2;});QTest::qWait(250);
      auto first_line=state["points"].toArray();
      if(std::abs(first_line[0].toArray()[0].toDouble()-view->subProp("X")->getValue().toDouble())>0.15 ||
         std::abs(first_line[0].toArray()[1].toDouble()-view->subProp("Y")->getValue().toDouble())>0.15)
        throw std::runtime_error("mouse center does not project to view center");
      if(std::abs(first_line[0].toArray()[1].toDouble()-first_line[1].toArray()[1].toDouble())>0.001)
        throw std::runtime_error("first straight segment is not horizontal despite equal click y");
      QTest::mouseMove(window,center+QPoint(80,60),40);
      QTest::mouseClick(window,Qt::LeftButton,Qt::NoModifier,center+QPoint(80,60));
      wait([&](){return state["points"].toArray().size()==3;});QTest::qWait(250);
      auto joined=state["points"].toArray();
      if(joined[0]!=first_line[0] || joined[1]!=first_line[1])throw std::runtime_error("continuous segment changed old endpoints");
      if(std::abs(joined[1].toArray()[0].toDouble()-joined[2].toArray()[0].toDouble())>0.001)
        throw std::runtime_error("second straight segment is not vertical despite equal click x");
      int joined_revision=state["revision"].toInt();
      QTest::mouseMove(window,center+QPoint(110,90),30);
      QTest::keyClick(manager->getRenderPanel(),Qt::Key_Escape);
      QTest::qWait(300);
      if(state["revision"].toInt()!=joined_revision || state["points"].toArray()!=joined)
        throw std::runtime_error("Escape changed committed straight segments");
      QTest::mouseClick(findButton("撤销"),Qt::LeftButton);
      wait([&](){return state["points"].toArray().size()==2;});
      QTest::mouseClick(findButton("重做"),Qt::LeftButton);
      wait([&](){return state["points"].toArray().size()==3;});
      QTest::mouseClick(findButton("预览／校验"),Qt::LeftButton);
      wait([&](){return state["status"].toString().startsWith("OFFLINE GEOMETRY PREVIEW");});
      QTest::qWait(250);
      std::cout<<"STRAIGHT_PASS: two clicks -> two vertices, wiggle ignored, chained 3 vertices, Escape, undo, redo, preview"<<std::endl;
      if(findButton("执行…")->isEnabled()||findButton("恢复…")->isEnabled())throw std::runtime_error("offline execution buttons enabled");
      // Screenshot is visual evidence of the real software-rendered RViz window, not a mockup.
      QTest::qWait(200);app.primaryScreen()->grabWindow(frame.winId()).save(argv[2]);
      std::cout<<"GUI_PASS: live distance, preview distance, zoom in all three tools, middle-button pan, drag, preview, Escape, undo, redo, add, move, delete, offline execution disabled"<<std::endl;
    }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;result=1;
      app.primaryScreen()->grabWindow(frame.winId()).save(argv[2]);}
    // Destruction rather than close avoids RViz's Save Configuration modal dialog.
  }
  sub.reset();node.reset();ros.shutdown();return result;
}

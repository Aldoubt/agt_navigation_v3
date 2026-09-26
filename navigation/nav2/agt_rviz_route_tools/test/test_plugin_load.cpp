#include <gtest/gtest.h>
#include <QApplication>
#include <QPushButton>
#include "agt_rviz_route_tools/chinese.hpp"
#include "agt_rviz_route_tools/route_metrics.hpp"
#include <pluginlib/class_loader.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/panel.hpp>
TEST(Plugins, OffscreenLoadAndConstruct) {
  int argc=1; char name[]="test";char * argv[]={name,nullptr}; QApplication app(argc,argv);
  pluginlib::ClassLoader<rviz_common::Tool> tools("rviz_common","rviz_common::Tool");
  auto straight=tools.createSharedInstance("agt_rviz_route_tools/StraightRoute");
  EXPECT_EQ(straight->getShortcutKey(),'l');
  auto draw=tools.createSharedInstance("agt_rviz_route_tools/DrawRoute");
  auto edit=tools.createSharedInstance("agt_rviz_route_tools/EditWaypoints");
  EXPECT_EQ(draw->getShortcutKey(),'d'); EXPECT_EQ(edit->getShortcutKey(),'e');
  pluginlib::ClassLoader<rviz_common::Panel> panels("rviz_common","rviz_common::Panel");
  auto panel=panels.createSharedInstance("agt_rviz_route_tools/Workbench");
  EXPECT_NE(panel,nullptr);
  bool found=false;
  for(auto * button:panel->findChildren<QPushButton*>())
    if(button->text()=="预览／校验")found=true;
  EXPECT_TRUE(found);
  EXPECT_EQ(agt_rviz_route_tools::chinese("DRAFT revision 8: preview required"), QString("草稿版本 8: 请先预览"));
  EXPECT_EQ(agt_rviz_route_tools::chinese("EDIT REJECTED: stale edit: reload current draft"), QString("编辑被拒绝: 编辑版本已过期，请重新加载当前草稿"));
  EXPECT_EQ(agt_rviz_route_tools::chinese("CUSTOM_DIAGNOSTIC"), QString("CUSTOM_DIAGNOSTIC"));
}

TEST(Metrics, KnownPolylineAndEmpty) {
  auto zero=agt_rviz_route_tools::measureRoute(QJsonArray{});
  EXPECT_EQ(zero.count,0);EXPECT_DOUBLE_EQ(zero.total,0.0);
  auto single=agt_rviz_route_tools::measureRoute(QJsonArray{QJsonArray{2.,3.}});
  EXPECT_EQ(single.count,1);EXPECT_DOUBLE_EQ(single.total,0.0);
  auto route=agt_rviz_route_tools::measureRoute(QJsonArray{QJsonArray{0.,0.},QJsonArray{3.,0.},QJsonArray{3.,4.}});
  EXPECT_EQ(route.count,3);EXPECT_DOUBLE_EQ(route.total,7.0);EXPECT_DOUBLE_EQ(route.last,4.0);
  EXPECT_TRUE(agt_rviz_route_tools::metricsText(route,false).contains("7.00"));
}

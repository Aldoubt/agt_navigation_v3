#pragma once
#include <cmath>
#include <QJsonArray>
#include <QString>

namespace agt_rviz_route_tools {
struct RouteMetrics {double total=0.0; double last=0.0; int count=0;};
inline RouteMetrics measureRoute(const QJsonArray & points) {
  RouteMetrics m;
  double x0=0,y0=0;
  for(const auto & value:points) {
    auto p=value.toArray();
    if(p.size()!=2 || !p[0].isDouble() || !p[1].isDouble())return {};
    double x=p[0].toDouble(),y=p[1].toDouble();
    if(!std::isfinite(x)||!std::isfinite(y))return {};
    if(m.count){m.last=std::hypot(x-x0,y-y0);m.total+=m.last;}
    x0=x;y0=y;++m.count;
  }
  return m;
}
inline QString metricsText(const RouteMetrics & m,bool drawing) {
  return QString("%1总长：%2 米｜点数：%3\n末段长度：%4 米（不含起点连接段）")
    .arg(drawing?"正在绘制 · ":"草稿 · ").arg(m.total,0,'f',2).arg(m.count).arg(m.last,0,'f',2);
}
}

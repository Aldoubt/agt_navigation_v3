#pragma once
#include <algorithm>
#include <QString>
#include <vector>
#include <utility>

namespace agt_rviz_route_tools {
// Presentation only. ROS state strings, topic/service names and plugin IDs stay stable.
inline QString chinese(QString text) {
  static const auto translations = [] {
    std::vector<std::pair<QString, QString>> items = {
      {"OFFLINE GEOMETRY PREVIEW: no collision validation; execution disabled", "离线几何预览：未进行碰撞校验，禁止执行"},
      {"DRAFT: drag a stroke or edit waypoints; preview before execution", "草稿：可拖动画线或编辑航点，执行前请先预览"},
      {"MAP UPDATED: preview required", "地图已更新：请重新预览"},
      {"VALIDATING: wait for VALIDATED before confirming execution", "正在校验：请等待校验通过后再确认执行"},
      {"VALIDATED: confirm Execute (or Resume for paused route)", "校验通过：请确认执行；已暂停的路线请选择恢复"},
      {"PAUSED: Preview remaining route, then confirm Resume; editing locked until Cancel", "已暂停：请先预览剩余路线，再确认恢复；取消任务后才能编辑"},
      {"PENDING: awaiting Navigation Capability acceptance", "正在提交：等待导航能力层接收"},
      {"PREVIEW EXPIRED: preview again", "预览已失效：请重新预览"},
      {"VALIDATION TIMEOUT: preview again", "校验超时：请重新预览"},
      {"CANCEL NOT ACKNOWLEDGED: task locked until terminal result; retry/stop safely", "取消未获确认：任务保持锁定；请重试或使用安全停车手段"},
      {"CANCELED / IDLE", "已取消／空闲"},
      {"offline mode: execution disabled; use Preview", "离线模式禁止执行，请使用预览"},
      {"fresh validated preview required; no active/pending task", "需要有效且已校验的预览，并且不能有活动或待接收的任务"},
      {"use Resume for a paused preview, Execute for a new route", "暂停路线请使用恢复，新路线请使用执行"},
      {"cancel active/paused route before editing", "请先取消正在运行或已暂停的路线，再进行编辑"},
      {"route locked: cancel active/paused route before editing", "路线已锁定：请先取消任务后再编辑"},
      {"cancel latched, including pending goal; wait for terminal result", "已记录取消请求（包括待接收目标），请等待任务终态"},
      {"pause requested; wait for confirmed PAUSED", "已请求暂停，请等待暂停确认"},
      {"confirmed path submitted unchanged", "已原样提交确认过的路径"},
      {"ambiguous crossing/loop at pause: cancel and redraw", "暂停点存在交叉或回环歧义，请取消任务后重新绘制"},
      {"robot moved after pause; cancel and redraw instead of ambiguous resume", "机器人在暂停后发生位移，请取消任务并重新绘制"},
      {"start connection exceeds limit; draw near robot or plan a connector", "起点连接距离超限，请在机器人附近绘制或先规划连接路线"},
      {"stale edit: reload current draft", "编辑版本已过期，请重新加载当前草稿"},
      {"nonfinite or out-of-bounds vertex", "顶点包含无效数值或坐标越界"},
      {"too many vertices (maximum 2000)", "顶点过多（最多 2000 个）"},
      {"route exceeds 200 m", "路线长度超过 200 米"},
      {"route spacing must be finite and between 0.01 and 0.05 m", "路径采样间距必须为有效数值，且在 0.01～0.05 米之间"},
      {"path too short or too many samples", "路径过短或采样点过多"},
      {"path collision/invalid costmap poses", "路径存在碰撞或代价地图中的无效位置"},
      {"health changed during validation", "校验期间导航健康状态发生变化"},
      {"robot moved during validation", "校验期间机器人发生位移"},
      {"robot moved: preview again", "机器人位置或朝向已变化，请重新预览"},
      {"robot TF stale: fresh localization required", "机器人 TF 已过期，需要有效的实时定位"},
      {"map -> base_link unavailable", "无法获取地图到机器人底盘的坐标变换"},
      {"preview revision/map changed", "预览版本或地图已变化"},
      {"navigation health not READY", "导航健康状态未就绪"},
      {"Navigation Capability rejected goal", "导航能力层拒绝了目标"},
      {"Navigation Capability unavailable", "导航能力层不可用"},
      {"IsPathValid unavailable", "路径校验服务不可用"},
      {"edit requires an object containing a point array", "编辑消息必须包含顶点数组"},
      {"integer revision required", "草稿版本号必须是整数"},
      {"edit requires map frame", "编辑必须使用地图坐标系"},
      {"map frame required", "需要地图坐标系"},
      {"edit message too large", "编辑消息过大"},
      {"each vertex requires x,y", "每个顶点必须包含 x、y 坐标"},
      {"no active or pending route", "没有正在运行或待接收的路线"},
      {"route active or pending", "路线正在运行或等待接收"},
      {"map not received", "尚未收到地图"},
      {"draw a route first", "请先绘制路线"},
      {"nothing to undo", "没有可撤销的操作"},
      {"nothing to redo", "没有可重做的操作"},
      {"preview expired", "预览已过期"},
      {"trajectory trails cleared", "历史轨迹已清空"},
      {"LIO AND WHEEL TRAILS CLEARED", "激光里程计和轮速轨迹已清空"},
      {"remain locked, verify stop independently", "任务保持锁定，请独立确认车辆已停车"},
      {"remain locked, verify stop before restarting tool", "任务保持锁定，重启工具前必须确认车辆已停车"},
      {"task remains locked", "任务保持锁定"},
      {"remain locked", "保持锁定"},
      {"ACCEPTED GOAL STATE UNKNOWN", "已接收目标的状态未知"},
      {"SUBMIT STATE UNKNOWN", "提交状态未知"},
      {"RESULT UNKNOWN", "任务结果未知"},
      {"CANCEL ACK ERROR", "取消确认异常"},
      {"CANCEL ERROR", "取消异常"},
      {"SUBMIT FAILED", "提交失败"},
      {"EDIT REJECTED", "编辑被拒绝"},
      {"PREVIEW REJECTED", "预览被拒绝"},
      {"EXECUTION REJECTED", "执行被拒绝"},
      {"VALIDATION FAILED", "校验失败"},
      {"DRAFT revision ", "草稿版本 "},
      {"preview required", "请先预览"},
      {"FINISHED status=4", "任务结束（成功）"},
      {"FINISHED status=5", "任务结束（已取消）"},
      {"FINISHED status=6", "任务结束（已中止）"},
      {"UNAVAILABLE", "不可用"},
      {"NOT_READY", "未就绪"},
      {"DEGRADED", "降级"},
      {"LOCALIZING", "正在定位"},
      {"LOCALIZED", "已定位"},
      {"PAUSING", "正在暂停"},
      {"CANCELING", "正在取消"},
      {"RUNNING", "正在执行"},
      {"READY", "就绪"},
      {"BUSY", "忙碌"},
      {"ERROR", "错误"}
    };
    std::stable_sort(items.begin(), items.end(), [](const auto &a, const auto &b) {return a.first.size()>b.first.size();});
    return items;
  }();
  for (const auto & item: translations) text.replace(item.first, item.second);
  return text; // Preserve unrecognized diagnostics rather than hiding useful error details.
}
inline QString actionChinese(const QString & key) {
  if(key=="preview")return "预览／校验";
  if(key=="start")return "执行";
  if(key=="resume")return "恢复";
  if(key=="pause")return "暂停";
  if(key=="cancel")return "取消任务";
  if(key=="undo")return "撤销";
  if(key=="redo")return "重做";
  if(key=="clear_route")return "清空草稿";
  if(key=="clear_trails")return "清空轨迹";
  return key;
}
}

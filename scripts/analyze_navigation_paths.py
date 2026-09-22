#!/usr/bin/env python3
"""Overlay Nav2 plans, local LIO odometry, and wheel odometry on a costmap.

The wheel pose is a separate odometry origin, so it is rigidly aligned to the
first map-frame LIO pose. This preserves its measured relative trajectory while
making the three route sources directly comparable.
"""

from __future__ import annotations

import argparse
import base64
import bisect
import csv
import io
import json
import math
import sqlite3
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {'/tf', '/agt/odometry/local', '/wheel/odom', '/plan'}


def yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def compose_2d(a, b):
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return ax + c * bx - s * by, ay + s * bx + c * by, at + bt


def inverse_2d(p):
    x, y, theta = p
    c, s = math.cos(theta), math.sin(theta)
    return -c * x - s * y, s * x - c * y, -theta


def nearest(rows, times, stamp):
    if not rows:
        return None
    index = bisect.bisect_left(times, stamp)
    candidates = []
    if index < len(rows):
        candidates.append(rows[index])
    if index:
        candidates.append(rows[index - 1])
    return min(candidates, key=lambda item: abs(item[0] - stamp))


def read_bag(bag: Path):
    plans, local, wheel, map_odom = [], [], [], []
    first_global_costmap = None
    message_types = {}
    bag_start = None

    for db_path in sorted(bag.glob('*.db3')):
        database = sqlite3.connect(str(db_path))
        topics = {
            topic_id: (name, type_name)
            for topic_id, name, type_name in database.execute(
                'select id,name,type from topics')
        }
        for _, (name, type_name) in topics.items():
            message_types[name] = type_name

        if first_global_costmap is None:
            cost_topic = next(
                (topic_id for topic_id, (name, _) in topics.items()
                 if name == '/global_costmap/costmap'), None)
            if cost_topic is not None:
                row = database.execute(
                    'select timestamp,data from messages where topic_id=? '
                    'order by timestamp limit 1', (cost_topic,)).fetchone()
                if row:
                    type_name = topics[cost_topic][1]
                    first_global_costmap = (
                        row[0] / 1e9,
                        deserialize_message(row[1], get_message(type_name)),
                    )

        selected_ids = [
            topic_id for topic_id, (name, _) in topics.items() if name in TOPICS
        ]
        if not selected_ids:
            database.close()
            continue
        placeholders = ','.join('?' for _ in selected_ids)
        query = (
            'select timestamp,topic_id,data from messages '
            f'where topic_id in ({placeholders}) order by timestamp'
        )
        classes = {
            topic_id: get_message(topics[topic_id][1]) for topic_id in selected_ids
        }
        for timestamp_ns, topic_id, blob in database.execute(query, selected_ids):
            stamp = timestamp_ns / 1e9
            bag_start = stamp if bag_start is None else min(bag_start, stamp)
            name = topics[topic_id][0]
            message = deserialize_message(blob, classes[topic_id])
            if name == '/tf':
                for transform in message.transforms:
                    parent = transform.header.frame_id.lstrip('/')
                    child = transform.child_frame_id.lstrip('/')
                    if parent == 'map' and child == 'odom':
                        value = transform.transform
                        map_odom.append((
                            stamp, value.translation.x, value.translation.y,
                            yaw(value.rotation),
                        ))
            elif name == '/agt/odometry/local':
                pose = message.pose.pose
                local.append((
                    stamp, pose.position.x, pose.position.y,
                    yaw(pose.orientation), message.twist.twist.linear.x,
                    message.twist.twist.angular.z,
                ))
            elif name == '/wheel/odom':
                pose = message.pose.pose
                wheel.append((
                    stamp, pose.position.x, pose.position.y,
                    yaw(pose.orientation), message.twist.twist.linear.x,
                    message.twist.twist.angular.z,
                ))
            elif name == '/plan' and message.poses:
                points = [
                    (pose.pose.position.x, pose.pose.position.y)
                    for pose in message.poses
                ]
                yaws = [yaw(pose.pose.orientation) for pose in message.poses]
                plans.append({'time': stamp, 'points': points, 'yaws': yaws})
        database.close()

    map_odom.sort()
    local.sort()
    wheel.sort()
    plans.sort(key=lambda item: item['time'])
    return bag_start, map_odom, local, wheel, plans, first_global_costmap


def map_lio_trajectory(map_odom, local):
    correction_times = [row[0] for row in map_odom]
    output = []
    for row in local:
        correction = nearest(map_odom, correction_times, row[0])
        if correction is None:
            continue
        pose = compose_2d(correction[1:4], row[1:4])
        output.append((row[0], *pose, row[4], row[5]))
    return output


def aligned_wheel_trajectory(wheel, lio_map):
    if not wheel or not lio_map:
        return []
    lio_times = [row[0] for row in lio_map]
    reference_lio = nearest(lio_map, lio_times, wheel[0][0])
    map_wheel_origin = compose_2d(reference_lio[1:4], inverse_2d(wheel[0][1:4]))
    return [
        (row[0], *compose_2d(map_wheel_origin, row[1:4]), row[4], row[5])
        for row in wheel
    ]


def costmap_array(costmap_message):
    info = costmap_message.info
    values = np.asarray(costmap_message.data, dtype=np.int16).reshape(
        int(info.height), int(info.width))
    return {
        'values': values,
        'width': int(info.width),
        'height': int(info.height),
        'resolution': float(info.resolution),
        'origin_x': float(info.origin.position.x),
        'origin_y': float(info.origin.position.y),
    }


def sample_cost(costmap, x, y):
    column = int(math.floor((x - costmap['origin_x']) / costmap['resolution']))
    row = int(math.floor((y - costmap['origin_y']) / costmap['resolution']))
    if row < 0 or column < 0 or row >= costmap['height'] or column >= costmap['width']:
        return -2
    return int(costmap['values'][row, column])


def approximate_clearance(cost):
    # OccupancyGrid conversion maps inflation costs 1..252 to roughly 1..98.
    # The rectangular footprint's padded inscribed radius is 0.43 m and the
    # configured inflation decay factor is 2.0.
    if cost == 0:
        return 1.0
    if 1 <= cost <= 98:
        raw_cost = max(1.0, cost * 254.0 / 100.0)
        return 0.43 - math.log(raw_cost / 253.0) / 2.0
    if cost in (99, 100):
        return 0.43
    return None


def route_cost_stats(points, costmap):
    costs = [sample_cost(costmap, x, y) for x, y in points]
    known = [cost for cost in costs if cost >= 0]
    clearances = [approximate_clearance(cost) for cost in known]
    clearances = [value for value in clearances if value is not None]
    count = len(costs) or 1
    return {
        'point_count': len(costs),
        'unknown_fraction': sum(cost == -1 for cost in costs) / count,
        'outside_fraction': sum(cost == -2 for cost in costs) / count,
        'inflated_fraction': sum(1 <= cost <= 98 for cost in costs) / count,
        'inscribed_or_lethal_fraction': sum(cost >= 99 for cost in costs) / count,
        'clearance_p10_m': float(np.quantile(clearances, 0.10)) if clearances else None,
        'clearance_p50_m': float(np.quantile(clearances, 0.50)) if clearances else None,
        'clearance_below_0_70_fraction': (
            sum(value < 0.70 for value in clearances) / len(clearances)
            if clearances else None),
    }


def footprint_collision_stats(plan, costmap):
    # Costmap2D applies footprint_padding=0.03 to the 1.04 x 0.80 m rectangle.
    vertices = ((0.55, 0.43), (0.55, -0.43), (-0.55, -0.43), (-0.55, 0.43))
    sample_step = costmap['resolution'] * 0.5
    checked = 0
    colliding = 0
    first_collision = None
    for point_index, ((x, y), theta) in enumerate(zip(plan['points'], plan['yaws'])):
        c, s = math.cos(theta), math.sin(theta)
        transformed = [
            (x + c * px - s * py, y + s * px + c * py) for px, py in vertices
        ]
        collision = False
        for start, end in zip(transformed, transformed[1:] + transformed[:1]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            steps = max(1, int(math.ceil(length / sample_step)))
            for step in range(steps + 1):
                ratio = step / steps
                sx = start[0] + ratio * (end[0] - start[0])
                sy = start[1] + ratio * (end[1] - start[1])
                if sample_cost(costmap, sx, sy) >= 100:
                    collision = True
                    break
            if collision:
                break
        checked += 1
        if collision:
            colliding += 1
            if first_collision is None:
                first_collision = point_index
    return {
        'colliding_pose_fraction': colliding / checked if checked else None,
        'first_colliding_pose_index': first_collision,
    }


def simplify(rows, min_distance=0.03, max_period=0.5):
    output = []
    for row in rows:
        if not output:
            output.append(row)
            continue
        if (math.hypot(row[1] - output[-1][1], row[2] - output[-1][2]) >= min_distance
                or row[0] - output[-1][0] >= max_period):
            output.append(row)
    return output


def path_deviation(lio_map, plans):
    if not plans:
        return []
    plan_times = [item['time'] for item in plans]
    arrays = [np.asarray(item['points'], dtype=float) for item in plans]
    output = []
    for row in lio_map:
        if abs(row[4]) < 0.05 or row[0] < plan_times[0]:
            continue
        index = bisect.bisect_right(plan_times, row[0]) - 1
        points = arrays[index]
        if not len(points):
            continue
        distance = np.sqrt(np.min(
            (points[:, 0] - row[1]) ** 2 + (points[:, 1] - row[2]) ** 2))
        output.append((row[0], float(distance), index))
    return output


def costmap_image(costmap):
    values = costmap['values']
    rgb = np.zeros((costmap['height'], costmap['width'], 3), dtype=np.uint8)
    rgb[values == -1] = (116, 132, 132)
    rgb[values == 0] = (250, 250, 248)
    inflation = (values >= 1) & (values <= 98)
    level = np.clip(values.astype(float) / 98.0, 0.0, 1.0)
    rgb[..., 0][inflation] = (120 + 125 * level[inflation]).astype(np.uint8)
    rgb[..., 1][inflation] = (235 - 125 * level[inflation]).astype(np.uint8)
    rgb[..., 2][inflation] = (235 - 45 * level[inflation]).astype(np.uint8)
    rgb[values == 99] = (104, 0, 92)
    rgb[values >= 100] = (28, 18, 30)
    return Image.fromarray(np.flipud(rgb), mode='RGB')


def crop_geometry(costmap, plans, lio, wheel, margin_m=3.0):
    points = []
    for plan in plans:
        points.extend(plan['points'])
    points.extend((row[1], row[2]) for row in lio)
    points.extend((row[1], row[2]) for row in wheel)
    if not points:
        return 0, 0, costmap['width'], costmap['height']
    xs, ys = zip(*points)
    resolution = costmap['resolution']
    min_col = max(0, int((min(xs) - margin_m - costmap['origin_x']) / resolution))
    max_col = min(
        costmap['width'],
        int((max(xs) + margin_m - costmap['origin_x']) / resolution) + 1)
    min_row = max(0, int((min(ys) - margin_m - costmap['origin_y']) / resolution))
    max_row = min(
        costmap['height'],
        int((max(ys) + margin_m - costmap['origin_y']) / resolution) + 1)
    # PIL coordinates use top-left origin.
    return min_col, costmap['height'] - max_row, max_col, costmap['height'] - min_row


def to_crop_pixel(costmap, crop, x, y):
    column = (x - costmap['origin_x']) / costmap['resolution'] - crop[0]
    image_row = (costmap['height'] - 1 -
                 (y - costmap['origin_y']) / costmap['resolution']) - crop[1]
    return float(column), float(image_row)


def draw_static(output_path, background, costmap, crop, plans, lio, wheel):
    canvas = background.convert('RGBA')
    plan_layer = Image.new('RGBA', canvas.size, (0, 0, 0, 0))
    plan_draw = ImageDraw.Draw(plan_layer)
    for plan in plans:
        pixels = [to_crop_pixel(costmap, crop, *point) for point in plan['points']]
        if len(pixels) >= 2:
            plan_draw.line(pixels, fill=(255, 55, 40, 55), width=3)
    canvas = Image.alpha_composite(canvas, plan_layer)
    draw = ImageDraw.Draw(canvas)
    lio_pixels = [to_crop_pixel(costmap, crop, row[1], row[2]) for row in lio]
    wheel_pixels = [to_crop_pixel(costmap, crop, row[1], row[2]) for row in wheel]
    if len(wheel_pixels) >= 2:
        draw.line(wheel_pixels, fill=(33, 180, 50, 235), width=5)
    if len(lio_pixels) >= 2:
        draw.line(lio_pixels, fill=(0, 105, 255, 255), width=5)
    if plans:
        latest_by_goal = {}
        for plan in plans:
            goal = tuple(round(value, 1) for value in plan['points'][-1])
            latest_by_goal[goal] = plan
        for plan in latest_by_goal.values():
            pixels = [to_crop_pixel(costmap, crop, *point) for point in plan['points']]
            if len(pixels) >= 2:
                draw.line(pixels, fill=(255, 120, 0, 255), width=4)
    if lio_pixels:
        radius = 7
        for point, color in ((lio_pixels[0], (20, 20, 20, 255)),
                             (lio_pixels[-1], (255, 220, 0, 255))):
            draw.ellipse((point[0] - radius, point[1] - radius,
                          point[0] + radius, point[1] + radius), fill=color)

    header_height = 76
    final = Image.new('RGB', (canvas.width, canvas.height + header_height), (245, 245, 245))
    final.paste(canvas.convert('RGB'), (0, header_height))
    header = ImageDraw.Draw(final)
    try:
        title_font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 23)
        legend_font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
    except OSError:
        title_font = legend_font = None
    header.text((16, 9), 'Nav2 plans and measured trajectories (map frame)',
                fill=(25, 25, 25), font=title_font)
    entries = [
        ((255, 120, 0), 'latest plan per goal'),
        ((255, 55, 40), 'all replans'),
        ((0, 105, 255), 'FAST-LIO2'),
        ((33, 180, 50), 'wheel odometry aligned at start'),
    ]
    x = 18
    for color, label in entries:
        header.line((x, 57, x + 30, 57), fill=color, width=5)
        header.text((x + 37, 46), label, fill=(35, 35, 35), font=legend_font)
        x += 37 + len(label) * 9 + 34
    final.save(output_path)


def html_explorer(output_path, background, costmap, crop, plans, lio, wheel, plan_stats, t0):
    image_buffer = io.BytesIO()
    background.save(image_buffer, format='PNG')
    image_uri = 'data:image/png;base64,' + base64.b64encode(image_buffer.getvalue()).decode()

    def pixels(points):
        return [[round(a, 2), round(b, 2)] for a, b in (
            to_crop_pixel(costmap, crop, *point) for point in points)]

    payload = {
        'width': background.width,
        'height': background.height,
        'resolution': costmap['resolution'],
        'origin_x': costmap['origin_x'] + crop[0] * costmap['resolution'],
        'origin_y': costmap['origin_y'] + (costmap['height'] - crop[3]) * costmap['resolution'],
        'plans': [
            {'time': round(plan['time'] - t0, 2), 'points': pixels(plan['points']),
             'goal': [round(value, 2) for value in plan['points'][-1]],
             'stats': plan_stats[index]}
            for index, plan in enumerate(plans)
        ],
        'lio': pixels([(row[1], row[2]) for row in lio]),
        'wheel': pixels([(row[1], row[2]) for row in wheel]),
    }
    template = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>AGT navigation path explorer</title>
<style>
body{margin:0;font:14px system-ui;background:#202426;color:#eee;overflow:hidden}
#bar{height:84px;padding:8px 14px;box-sizing:border-box;background:#303638}
label{margin-right:16px} input[type=range]{width:320px;vertical-align:middle}
#info{margin-top:7px;color:#d9e8e8}.orange{color:#ff8c22}.blue{color:#35a2ff}.green{color:#43d060}
#view{display:block;cursor:grab;background:#555}
</style></head><body><div id="bar">
<label><input id="all" type="checkbox">全部重规划</label>
<label><input id="selected" type="checkbox" checked><span class="orange">当前全局路径</span></label>
<label><input id="lio" type="checkbox" checked><span class="blue">FAST-LIO2 实际轨迹</span></label>
<label><input id="wheel" type="checkbox" checked><span class="green">轮速实际轨迹（起点对齐）</span></label>
<label>路径 <input id="slider" type="range" min="0" value="0"></label>
<div id="info"></div></div><canvas id="view"></canvas>
<script>
const DATA=__PAYLOAD__, BG='__IMAGE__';
const c=document.getElementById('view'),x=c.getContext('2d'),img=new Image();
const slider=document.getElementById('slider');
slider.max=Math.max(0,DATA.plans.length-1);
slider.value=slider.max;
let scale=1,ox=0,oy=0,drag=false,lx=0,ly=0;
function line(points,color,width,alpha=1){
  if(points.length<2)return;
  x.save();x.globalAlpha=alpha;x.strokeStyle=color;x.lineWidth=width/scale;
  x.beginPath();x.moveTo(points[0][0],points[0][1]);
  for(let i=1;i<points.length;i++)x.lineTo(points[i][0],points[i][1]);
  x.stroke();x.restore();
}
function render(){
  c.width=innerWidth;c.height=innerHeight-84;
  x.setTransform(scale,0,0,scale,ox,oy);x.drawImage(img,0,0);
  if(all.checked)DATA.plans.forEach(p=>line(p.points,'#ff3e32',2,.20));
  const i=+slider.value,p=DATA.plans[i];
  if(selected.checked&&p)line(p.points,'#ff8700',4,1);
  if(wheel.checked)line(DATA.wheel,'#28c64d',4,.95);
  if(lio.checked)line(DATA.lio,'#087eff',4,1);
  if(p){const s=p.stats;info.textContent=
    `路径 ${i+1}/${DATA.plans.length}  t=${p.time}s  `+
    `目标=(${p.goal[0]}, ${p.goal[1]})  `+
    `未知区=${(100*s.unknown_fraction).toFixed(1)}%  `+
    `膨胀区=${(100*s.inflated_fraction).toFixed(1)}%  `+
    `<0.70m=${(100*(s.clearance_below_0_70_fraction||0)).toFixed(1)}%`;}
}
img.onload=()=>{
  scale=Math.min((innerWidth-20)/DATA.width,(innerHeight-104)/DATA.height);
  ox=(innerWidth-DATA.width*scale)/2;oy=10;render();
};
img.src=BG;
for(const id of ['all','selected','lio','wheel','slider']){
  document.getElementById(id).oninput=render;
}
c.onwheel=e=>{
  e.preventDefault();const old=scale;scale*=e.deltaY<0?1.15:1/1.15;
  scale=Math.max(.1,Math.min(12,scale));
  ox=e.offsetX-(e.offsetX-ox)*scale/old;
  oy=e.offsetY-(e.offsetY-oy)*scale/old;render();
};
c.onmousedown=e=>{drag=true;lx=e.clientX;ly=e.clientY;c.style.cursor='grabbing'};
onmouseup=()=>{drag=false;c.style.cursor='grab'};
onmousemove=e=>{
  if(drag){ox+=e.clientX-lx;oy+=e.clientY-ly;lx=e.clientX;ly=e.clientY;render();}
};
onresize=render;
</script></body></html>'''
    output_path.write_text(
        template.replace('__PAYLOAD__', json.dumps(payload, ensure_ascii=False))
        .replace('__IMAGE__', image_uri), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    t0, map_odom, local, wheel, plans, costmap_entry = read_bag(args.bag)
    if costmap_entry is None:
        raise RuntimeError('bag contains no /global_costmap/costmap message')
    costmap = costmap_array(costmap_entry[1])
    lio_map = map_lio_trajectory(map_odom, local)
    wheel_map = aligned_wheel_trajectory(wheel, lio_map)
    lio_draw = simplify(lio_map)
    wheel_draw = simplify(wheel_map)

    plan_stats = []
    for plan in plans:
        stats = route_cost_stats(plan['points'], costmap)
        stats.update(footprint_collision_stats(plan, costmap))
        plan_stats.append(stats)
    lio_stats = route_cost_stats([(row[1], row[2]) for row in lio_draw], costmap)
    wheel_stats = route_cost_stats([(row[1], row[2]) for row in wheel_draw], costmap)
    deviation = path_deviation(lio_map, plans)

    wheel_times = [row[0] for row in wheel_map]
    wheel_lio_separation = []
    for row in lio_map:
        if abs(row[4]) < 0.05:
            continue
        wheel_pose = nearest(wheel_map, wheel_times, row[0])
        if wheel_pose is not None:
            wheel_lio_separation.append(math.hypot(
                wheel_pose[1] - row[1], wheel_pose[2] - row[2]))

    endpoints = Counter(
        tuple(round(value, 1) for value in plan['points'][-1]) for plan in plans)
    intervals = [b['time'] - a['time'] for a, b in zip(plans, plans[1:])
                 if tuple(round(v, 1) for v in a['points'][-1]) ==
                 tuple(round(v, 1) for v in b['points'][-1])]
    goal_statistics = []
    for goal, plan_count in endpoints.items():
        members = [
            stats for plan, stats in zip(plans, plan_stats)
            if tuple(round(value, 1) for value in plan['points'][-1]) == goal
        ]
        goal_statistics.append({
            'goal': list(goal),
            'plans': plan_count,
            'unknown_fraction_median': float(np.median([
                item['unknown_fraction'] for item in members])),
            'inflated_fraction_median': float(np.median([
                item['inflated_fraction'] for item in members])),
            'clearance_below_0_70_fraction_median': float(np.median([
                item['clearance_below_0_70_fraction'] for item in members
                if item['clearance_below_0_70_fraction'] is not None])),
            'plans_with_footprint_collision': sum(
                item['colliding_pose_fraction'] > 0 for item in members),
        })
    summary = {
        'bag': str(args.bag),
        'map_yaml': str(args.map_yaml),
        'duration_s': max(
            [row[0] for row in local + wheel] + [plan['time'] for plan in plans]) - t0,
        'counts': {
            'plans': len(plans), 'map_to_odom': len(map_odom),
            'lio_samples': len(lio_map), 'wheel_samples': len(wheel_map),
        },
        'plan_endpoint_counts': [
            {'goal': list(goal), 'plans': count} for goal, count in endpoints.items()],
        'per_goal': goal_statistics,
        'same_goal_replan_interval_s': {
            'p50': float(np.quantile(intervals, .5)) if intervals else None,
            'p95': float(np.quantile(intervals, .95)) if intervals else None,
            'min': min(intervals) if intervals else None,
        },
        'all_plan_points': route_cost_stats(
            [point for plan in plans for point in plan['points']], costmap),
        'per_plan': plan_stats,
        'lio_route': lio_stats,
        'wheel_route': wheel_stats,
        'wheel_to_lio_separation_m': {
            'samples': len(wheel_lio_separation),
            'p50': float(np.quantile(wheel_lio_separation, .5))
            if wheel_lio_separation else None,
            'p95': float(np.quantile(wheel_lio_separation, .95))
            if wheel_lio_separation else None,
            'max': max(wheel_lio_separation, default=None),
        },
        'plans_with_footprint_collision': sum(
            stats['colliding_pose_fraction'] > 0 for stats in plan_stats),
        'lio_to_active_plan_deviation_m': {
            'samples': len(deviation),
            'p50': float(np.quantile([row[1] for row in deviation], .5)) if deviation else None,
            'p95': float(np.quantile([row[1] for row in deviation], .95)) if deviation else None,
            'max': max((row[1] for row in deviation), default=None),
        },
        'wheel_alignment': 'SE(2) aligned to the first map-frame FAST-LIO2 pose',
        'clearance_note': (
            'Clearance is inferred from the recorded global inflation cost using '
            'inscribed_radius=0.43m and cost_scaling_factor=2.0; free cost 0 is '
            'reported as at least the 1.0m inflation radius.'),
    }
    (args.output_dir / 'navigation_path_analysis.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')

    with (args.output_dir / 'trajectory_points.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source', 'seconds', 'map_x', 'map_y', 'yaw', 'linear_x', 'angular_z'])
        for source, rows in (('fastlio2', lio_draw), ('wheel_aligned', wheel_draw)):
            for row in rows:
                writer.writerow([source, row[0] - t0, *row[1:]])
    with (args.output_dir / 'global_plans.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['plan_index', 'seconds', 'point_index', 'map_x', 'map_y', 'global_cost'])
        for plan_index, plan in enumerate(plans):
            for point_index, point in enumerate(plan['points']):
                writer.writerow([
                    plan_index, plan['time'] - t0, point_index, *point,
                    sample_cost(costmap, *point),
                ])

    full_background = costmap_image(costmap)
    crop = crop_geometry(costmap, plans, lio_draw, wheel_draw)
    background = full_background.crop(crop)
    draw_static(args.output_dir / 'trajectory_overlay.png', background, costmap, crop,
                plans, lio_draw, wheel_draw)
    html_explorer(args.output_dir / 'trajectory_explorer.html', background, costmap, crop,
                  plans, lio_draw, wheel_draw, plan_stats, t0)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()

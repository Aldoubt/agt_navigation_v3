"""Real Humble ROS transport, lab mock Map Server only, zero robot motion.

Boot -> STALE/no route -> ASSET_READY+MAP_BOUND -> mutated same 'map' pixels
-> ASSET_READY+MAP_NOT_BOUND -> recovered -> invalid SHA -> restart -> STALE.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from std_msgs.msg import String


def run(config_path,good_path,bad_path,out):
    config=json.loads(Path(config_path).read_bytes())
    from agt_rviz_patrol.ready_route_resolver import _current
    from agt_rviz_patrol.map_view_binding import source_grid, _parse_p5, _cell_data
    import yaml
    _,summary=_current(config)
    site_root=Path(config["sites_root"])/config["site_id"]/config["site_revision"]
    expected=source_grid(site_root,summary)
    pgm=Path(expected["yaml_path"]).parent/"navigation.pgm"
    w,h,pixels=_parse_p5(pgm.read_bytes())
    cells=_cell_data(pixels,w,h,yaml.safe_load(Path(expected["yaml_path"]).read_bytes()))

    rclpy.init()
    server=Node("map_server")
    server.declare_parameter("yaml_filename",expected["yaml_path"])
    map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
    grid_pub=server.create_publisher(OccupancyGrid,"/map",map_qos)
    executor=SingleThreadedExecutor()
    executor.add_node(server)
    server_thread=threading.Thread(target=executor.spin,daemon=True)
    server_thread.start()

    client=Node("s7_2_qa_client")
    statuses=[]
    paths=[]
    client.create_subscription(String,"/agt/path_tool/ready_route_view/state",
                               lambda m:statuses.append(json.loads(m.data)),10)
    client.create_subscription(RosPath,"/agt/path_tool/ready_route_view/path",
                               lambda m:paths.append(len(m.poses)),10)
    sender=client.create_publisher(String,"/agt/path_tool/ready_route_view/request",10)
    exe=str(Path(get_package_prefix("agt_rviz_patrol")) /
            "lib/agt_rviz_patrol/ready_route_viewer")
    env=dict(os.environ,ROS_DOMAIN_ID="232",ROS_LOCALHOST_ONLY="1",
             AGT_S7_2_SYNTHETIC_MAP_ONLY="1")
    events=[]

    def spin_until(predicate,timeout=12):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(client,timeout_sec=0.15)
            if statuses and predicate(statuses[-1]):
                return dict(statuses[-1])
        raise AssertionError("timeout: "+repr(statuses[-4:]))

    def request(path,pred,expected_path):
        statuses.clear();paths.clear()
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            sender.publish(String(data=str(path)))
            rclpy.spin_once(client,timeout_sec=0.3)
            if statuses and pred(statuses[-1]):
                p_deadline=time.monotonic()+3
                while time.monotonic()<p_deadline and (
                    not paths or paths[-1]!=expected_path
                ):
                    rclpy.spin_once(client,timeout_sec=0.15)
                if paths and paths[-1]==expected_path:
                    return dict(statuses[-1])
        raise AssertionError("request failed: "+repr(statuses[-4:])+repr(paths[-4:]))

    def publish_grid(value):
        m=OccupancyGrid()
        m.header.frame_id="map"
        m.header.stamp=server.get_clock().now().to_msg()
        m.info.width=w;m.info.height=h;m.info.resolution=expected["resolution"]
        m.info.origin.position.x=expected["origin"][0]
        m.info.origin.position.y=expected["origin"][1]
        yaw=expected["origin"][2]
        m.info.origin.orientation.z=math.sin(yaw/2)
        m.info.origin.orientation.w=math.cos(yaw/2)
        m.data=list(value if value is not None else
                    (v if v<128 else v-256 for v in cells))
        grid_pub.publish(m)

    try:
        publish_grid(None)
        for boot in (1,2):
            with (out/f"s7_2_viewer_boot_{boot}.log").open("w") as log:
                proc=subprocess.Popen(
                    [exe,"--ros-args","-p",f"trusted_config:={config_path}",
                     "-p","offline_hard_lock:=true"],
                    env=env,stdout=log,stderr=subprocess.STDOUT,
                    start_new_session=True)
                try:
                    status=spin_until(lambda s:s["state"]=="STALE" and
                                      s["code"]=="NO_ROUTE_SELECTED")
                    events.append(dict(boot=boot,phase="fresh_stale",status=status,path=0))
                    good=request(good_path,lambda s:s["state"]=="MAP_BOUND_TO_VIEW",3)
                    events.append(dict(boot=boot,phase="map_bound",status=good,path=3))
                    assert good["asset_ready_verified"] and good["map_bound_to_view"]

                    publish_grid([100 for _ in cells])
                    bad=spin_until(lambda s:s.get("code")=="MAP_GRID_CONTENT_MISMATCH")
                    end=time.monotonic()+3
                    while time.monotonic()<end and (not paths or paths[-1]!=0):
                        rclpy.spin_once(client,timeout_sec=0.2)
                    assert paths and paths[-1]==0,paths[-4:]
                    assert bad["asset_ready_verified"] and not bad["map_bound_to_view"]
                    events.append(dict(boot=boot,phase="same_frame_different_grid",
                                       status=bad,path=0))
                    publish_grid(None)
                    recovered=request(good_path,
                                      lambda s:s["state"]=="MAP_BOUND_TO_VIEW",3)
                    events.append(dict(boot=boot,phase="map_rebound",
                                       status=recovered,path=3))
                    invalid=request(bad_path,lambda s:s["state"]=="INVALID",0)
                    events.append(dict(boot=boot,phase="bad_manifest_invalid",
                                       status=invalid,path=0))
                finally:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            statuses.clear();paths.clear()
        assert len(events)==10
        (out/"s7_2_ros_matrix.json").write_text(
            json.dumps(events,ensure_ascii=False,indent=2)+"\n")
        print("S7_2_FRESH_PROCESS_MAP_VIEW_ROS_PASS 10/10; PATH_ZERO_ON_MISMATCH; NO_MOTION")
    finally:
        client.destroy_node()
        executor.shutdown()
        server.destroy_node()
        server_thread.join(timeout=3)
        rclpy.shutdown()


if __name__=="__main__":
    run(Path(sys.argv[1]),Path(sys.argv[2]),Path(sys.argv[3]),Path(sys.argv[4]))

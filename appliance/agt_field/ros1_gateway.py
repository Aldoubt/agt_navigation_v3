"""Noetic host endpoint. Only standard Odometry/String/Bool and guarded Twist.

No CAN frames, LiDAR forwarding, TF publishing, or arbitrary topic discovery.
A vendor-specific status topic must be adapted by the ROS1 driver deployment.
"""

import argparse
import os
from pathlib import Path
import socket
import threading
import time

from .contracts import ContractError
from .gateway import Watchdog, Framer, encode
from .profile import load_profile, require_real


def ros1_dict(message):
    if hasattr(message, "secs") and hasattr(message, "nsecs"):
        return dict(sec=message.secs, nanosec=message.nsecs)
    if hasattr(message, "__slots__"):
        return {
            slot: ros1_dict(getattr(message, slot)) for slot in message.__slots__ if slot != "seq"
        }
    if isinstance(message, (list, tuple)):
        return [ros1_dict(v) for v in message]
    return message


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--data-root", required=True)
    args = parser.parse_args(argv)
    p = load_profile(args.profile)
    motion_enabled = True
    motion_error = ""
    try:
        require_real(p, motion=True)
    except ContractError as exc:
        motion_enabled = False
        motion_error = str(exc)
    for key in ["ros1_odom", "ros1_chassis", "ros1_estop"]:
        if not p["topics"].get(key):
            raise ContractError("CONFIG_REQUIRED: " + key)
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool

    rospy.init_node("agt_yhs_ros1_gateway", disable_signals=True)
    publisher = (
        rospy.Publisher(p["topics"]["ros1_cmd_vel"], Twist, queue_size=1)
        if p["topics"].get("ros1_cmd_vel")
        else None
    )
    if not motion_enabled:
        rospy.logwarn("Monitoring only; motion disabled: " + motion_error)
    lock = threading.RLock()
    # In monitor mode every output is zero; this housekeeping timer is NOT a hardware safety threshold.
    w = Watchdog(p["base"]["command_timeout_sec"] or 0.1)
    latest = {}
    estop_rx = [0.0]
    chassis_rx = [0.0]

    def odom(msg):
        with lock:
            latest["odom"] = dict(type="odom", message=ros1_dict(msg))

    def chassis(msg):
        with lock:
            latest["chassis"] = dict(type="chassis", message=msg.data)
            chassis_rx[0] = time.monotonic()

    def estop(msg):
        with lock:
            estop_rx[0] = time.monotonic()
            w.estop = bool(msg.data)
            latest["estop"] = dict(type="estop", value=w.estop)

    rospy.Subscriber(p["topics"]["ros1_odom"], Odometry, odom, queue_size=1)
    rospy.Subscriber(p["topics"]["ros1_chassis"], String, chassis, queue_size=1)
    rospy.Subscriber(p["topics"]["ros1_estop"], Bool, estop, queue_size=1)
    path = Path(args.data_root) / "run/gateway.sock"
    path.parent.mkdir(parents=True, exist_ok=True)
    # Refuse to unlink a socket that still has an active listener.
    if path.exists():
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            probe.connect(str(path))
        except ConnectionRefusedError:
            path.unlink()
        else:
            raise ContractError("ROS1 gateway already running")
        finally:
            probe.close()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    os.chmod(path, 0o600)
    server.listen(1)
    server.settimeout(0.1)
    client = None
    frame = Framer()
    session = None
    next_tick = time.monotonic()
    try:
        while not rospy.is_shutdown():
            if client is None:
                try:
                    client, _ = server.accept()
                    client.settimeout(0.005)
                    frame = Framer()
                    session = None
                except socket.timeout:
                    pass
            if client:
                try:
                    raw = client.recv(16384)
                    if not raw:
                        raise ConnectionError("EOF")
                    for row in frame.feed(raw):
                        with lock:
                            if row.get("type") == "hello" and session is None:
                                session = row["session"]
                                w.connect(session)
                            elif row.get("type") == "cmd_vel":
                                w.command(
                                    row["session"], row["sequence"], row["linear"], row["angular"]
                                )
                            else:
                                raise ContractError("invalid gateway record")
                except socket.timeout:
                    pass
                except (OSError, ValueError, KeyError):
                    with lock:
                        w.disconnect()
                    client.close()
                    client = None
            now = time.monotonic()
            if now >= next_tick:
                with lock:
                    if now - estop_rx[0] > 0.5 or now - chassis_rx[0] > 0.5:
                        w.estop = True
                    x, z = w.output() if motion_enabled else (0.0, 0.0)
                    msg = Twist()
                    msg.linear.x = x
                    msg.angular.z = z
                    if publisher:
                        publisher.publish(msg)
                    pending = [row for row in latest.values() if row["type"] != "estop"]
                    latest.clear()
                    pending.append(dict(type="estop", value=w.estop or not motion_enabled))
                    pending.append(
                        dict(
                            type="gateway_status",
                            motion_enabled=motion_enabled,
                            reason=motion_error,
                        )
                    )
                if client:
                    try:
                        client.sendall(b"".join(encode(row) for row in pending))
                    except OSError:
                        with lock:
                            w.disconnect()
                        client.close()
                        client = None
                next_tick += 0.02
                if next_tick < now - 0.02:
                    next_tick = now + 0.02
            time.sleep(0.001)
    finally:
        for _ in range(5):
            if publisher:
                publisher.publish(Twist())
            time.sleep(0.02)
        if client:
            client.close()
        server.close()
        path.unlink(missing_ok=True)
        rospy.signal_shutdown("gateway stopped; final zero published")


if __name__ == "__main__":
    main()

"""Noetic standard-message fixture. No CAN, TF, vendor driver or real robot."""

import json
import os
from pathlib import Path
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

rospy.init_node("software_yhs_driver_fixture")
trace = Path(os.environ["AGT_TEST_GATEWAY_ROOT"]) / "driver_twist.jsonl"


def command(message):
    with trace.open("a") as stream:
        stream.write(
            json.dumps(
                dict(
                    time=rospy.Time.now().to_sec(),
                    linear=message.linear.x,
                    angular=message.angular.z,
                )
            )
            + "\n"
        )


rospy.Subscriber("/test/driver/cmd_vel", Twist, command, queue_size=100)
odom = rospy.Publisher("/test/driver/odom", Odometry, queue_size=1)
chassis = rospy.Publisher("/test/driver/chassis", String, queue_size=1)
estop = rospy.Publisher("/test/driver/estop", Bool, queue_size=1)
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "test_wheel_odom"
    msg.child_frame_id = "test_base"
    msg.pose.pose.position.x = 2.4
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance = [0.3] * 36
    msg.twist.covariance = [0.2] * 36
    odom.publish(msg)
    chassis.publish(String(data='{"fixture":true}'))
    estop.publish(Bool(data=False))
    rate.sleep()

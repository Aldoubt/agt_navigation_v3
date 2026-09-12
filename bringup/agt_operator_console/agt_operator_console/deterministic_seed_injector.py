"""Test-side one-shot map-frame seed injector for offline replay only."""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

_CLOCK_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.BEST_EFFORT)

class DeterministicSeedInjector(Node):
    def __init__(self):
        super().__init__('deterministic_seed_injector')
        for n,v in {'target_clock_sec':0.0,'x':0.0,'y':0.0,'z':0.0,
                    'qx':0.0,'qy':0.0,'qz':0.0,'qw':1.0}.items(): self.declare_parameter(n,v)
        self.target=float(self.get_parameter('target_clock_sec').value); self.sent=False
        self.pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.create_subscription(Clock,'/clock',self.on_clock,_CLOCK_QOS)
    def on_clock(self,msg):
        now=msg.clock.sec+msg.clock.nanosec/1e9
        if self.sent or now < self.target: return
        p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=msg.clock
        q=p.pose.pose.orientation; pos=p.pose.pose.position
        pos.x=float(self.get_parameter('x').value);pos.y=float(self.get_parameter('y').value);pos.z=float(self.get_parameter('z').value)
        q.x=float(self.get_parameter('qx').value);q.y=float(self.get_parameter('qy').value);q.z=float(self.get_parameter('qz').value);q.w=float(self.get_parameter('qw').value)
        self.pub.publish(p);self.sent=True;self.get_logger().info(f'published deterministic seed at replay clock {now:.9f}')
def main(args=None):
    rclpy.init(args=args);n=DeterministicSeedInjector()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();
    if rclpy.ok():rclpy.shutdown()

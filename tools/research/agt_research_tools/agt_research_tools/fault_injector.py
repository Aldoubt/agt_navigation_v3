"""Fault injection on isolated research topics; never publishes TF or velocity."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from agt_navigation_interfaces.msg import GlobalQuality, LocalRowState, OdomQuality, RelocalizationCandidate


class FaultInjector(Node):
    def __init__(self):
        super().__init__('agt_research_fault_injector')
        for key,value in {'enabled':False,'fault':'C0','start_after_sec':0.0,'duration_sec':0.0,
                          'candidate_delta_x_m':0.0,'candidate_delta_y_m':0.0,
                          'candidate_delta_yaw_rad':0.0,
                          'output_prefix':'/agt/research/faults'}.items():
            self.declare_parameter(key,value)
        prefix=self.get_parameter('output_prefix').value
        if not isinstance(prefix,str) or not (prefix=='/agt/research/faults' or prefix.startswith('/agt/research/faults/')):
            raise ValueError('fault outputs must remain in /agt/research/faults namespace')
        if self.get_parameter('fault').value not in ('C0','C1','C2','C3','C4','C5'):
            raise ValueError('fault must be C0..C5')
        for key in ('start_after_sec','duration_sec','candidate_delta_x_m','candidate_delta_y_m','candidate_delta_yaw_rad'):
            val=float(self.get_parameter(key).value)
            if not math.isfinite(val) or (key in ('start_after_sec','duration_sec') and val<0):
                raise ValueError('finite fault parameters and nonnegative time bounds required')
        self.start=time.monotonic()
        for cls,topic,suffix in ((GlobalQuality,'/agt/localization/quality','global_quality'),
                                 (LocalRowState,'/agt/local_row/state','local_row'),
                                 (OdomQuality,'/agt/odometry/quality','odom_quality'),
                                 (RelocalizationCandidate,'/agt/localization/candidate','candidate')):
            pub=self.create_publisher(cls,prefix+'/'+suffix,10)
            self.create_subscription(cls,topic,lambda msg,pub=pub,suffix=suffix:self.forward(msg,pub,suffix),10)

    def forward(self,message,pub,suffix):
        elapsed=time.monotonic()-self.start
        start=float(self.get_parameter('start_after_sec').value)
        duration=float(self.get_parameter('duration_sec').value)
        active=(self.get_parameter('enabled').value and duration>0 and start<=elapsed<start+duration)
        fault=self.get_parameter('fault').value if active else 'C0'
        if fault=='C1' and suffix in ('global_quality','candidate'):
            return
        msg=copy.deepcopy(message)
        if fault in ('C2','C5') and suffix=='global_quality':
            msg.valid=False; msg.quality=0.0; msg.reason='research_fault_'+fault
        if fault=='C5' and suffix=='local_row':
            msg.row_valid=False; msg.clearance_valid=False; msg.quality=0.0
            msg.reasons.append('research_fault_C5')
        if fault=='C5' and suffix=='odom_quality':
            msg.valid=False; msg.quality=0.0; msg.reason='research_fault_C5'
        if fault=='C3' and suffix=='candidate':
            p=msg.pose.pose.pose.position; q=msg.pose.pose.pose.orientation
            p.x+=float(self.get_parameter('candidate_delta_x_m').value)
            p.y+=float(self.get_parameter('candidate_delta_y_m').value)
            angle=float(self.get_parameter('candidate_delta_yaw_rad').value)/2
            s,c=math.sin(angle),math.cos(angle)
            x,y,z,w=q.x,q.y,q.z,q.w
            q.x,q.y,q.z,q.w=c*x-s*y,c*y+s*x,c*z+s*w,c*w-s*z
            msg.quality.reason='research_fault_C3'
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node=FaultInjector()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

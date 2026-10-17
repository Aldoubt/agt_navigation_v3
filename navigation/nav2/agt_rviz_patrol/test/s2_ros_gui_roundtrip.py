"""S2 isolated offline GUI backend roundtrip against a launched RViz + map_server.

No motion action is sent. Only local draft edit/Trigger endpoints are used.
Run in stages (create, restore) separated by stopping/restarting offline launch.
"""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self):
        super().__init__('s2_offline_operator_probe')
        self.snapshot = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/agt/path_tool/editor_state', self.on_state, qos)
        self.pub = self.create_publisher(String, '/agt/path_tool/edit', 10)
        self._s2_clients = {}

    def on_state(self, msg):
        self.snapshot = json.loads(msg.data)

    def until(self, condition, label, timeout=15):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return
        raise AssertionError('timed out: '+label+'; last_state='+str(self.snapshot))

    def submit(self, points):
        previous = self.snapshot['revision']
        payload = dict(frame='map', revision=previous, points=points)
        self.pub.publish(String(data=json.dumps(payload)))
        self.until(lambda: self.snapshot and self.snapshot['revision'] > previous,
                   'edit revision '+str(previous))
        assert self.snapshot['points'] == [list(p) for p in points],self.snapshot
        print('EDIT',self.snapshot['revision'],self.snapshot['point_labels'], flush=True)

    def trigger(self, name, should_succeed=True):
        if name not in self._s2_clients:
            self._s2_clients[name] = self.create_client(Trigger, '/agt/path_tool/'+name)
        client = self._s2_clients[name]
        assert client.wait_for_service(timeout_sec=5), name+' unavailable'
        future = client.call_async(Trigger.Request())
        self.until(lambda: future.done(), 'service '+name)
        reply = future.result()
        assert reply.success == should_succeed, (name, reply.success, reply.message)
        print('TRIGGER',name,reply.success,reply.message,flush=True)
        return reply


def main(mode, filename):
    rclpy.init()
    probe = Probe()
    try:
        probe.until(lambda: probe.snapshot is not None and probe.snapshot['map_received'], 'map + state')
        assert probe.snapshot['preview_only'] is True
        assert probe.snapshot['draft_path'] == filename
        if mode=='create':
            assert probe.snapshot['points'] == []
            probe.submit([(0., 0.)])
            probe.submit([(0., 0.), (1., 0.)])
            probe.submit([(0., 0.), (1., 0.), (1., 1.)])
            probe.submit([(0., 0.), (1.1, .1), (1., 1.)])
            probe.trigger('undo')
            probe.until(lambda: probe.snapshot['points'][1] == [1., 0.], 'undo')
            probe.trigger('redo')
            probe.until(lambda: probe.snapshot['points'][1] == [1.1, .1], 'redo')
            probe.trigger('save_draft')
            stored = json.loads(Path(filename).read_text())
            assert stored['state']=='DRAFT' and len(stored['points'])==3
        else:
            assert probe.snapshot['points'] == []
            probe.trigger('load_draft')
            probe.until(lambda: len(probe.snapshot['points'])==3,'reloaded 3 points')
            assert probe.snapshot['points'][1] == [1.1,.1]
        probe.trigger('preview')
        probe.until(lambda: probe.snapshot['preview_length_m'] is not None, 'preview')
        assert probe.snapshot['can_execute'] is False
        probe.trigger('start',should_succeed=False)
        probe.trigger('resume',should_succeed=False)
        print('RESULT '+mode.upper()+': GUI_BACKEND_DRAFT_ROUNDTRIP PASS; ROUTE_REF UNBOUND; OFFLINE_NO_MOTION',flush=True)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('mode', choices=('create','restore'))
    parser.add_argument('filename')
    args=parser.parse_args()
    main(args.mode,args.filename)

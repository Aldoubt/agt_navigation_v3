#!/usr/bin/env python3
"""Map explicitly verified vendor status fields to one truthful BaseState."""
import math
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from agt_robot_interfaces.msg import BaseState


def field(message, path):
    if not isinstance(path, str) or not path:
        raise ValueError('confirmed field path required')
    for part in path.split('.'):
        if part.startswith('_'):
            raise ValueError('private field rejected')
        message = getattr(message, part)
    return message


def validate_mapping(spec):
    if not isinstance(spec,dict): raise ValueError('explicit status mapping required')
    if 'any_of' in spec:
        if not isinstance(spec['any_of'],list) or not spec['any_of']:
            raise ValueError('any_of requires explicit status fields')
        for item in spec['any_of']: validate_mapping(item)
        return
    if not spec.get('field'): raise ValueError('explicit field mapping required')
    if spec.get('predicate')=='nonzero': return
    active,inactive=spec.get('active_values'),spec.get('inactive_values')
    if (not isinstance(active,list) or not active or not isinstance(inactive,list) or not inactive
            or any(v in inactive for v in active)):
        raise ValueError('status needs disjoint active_values and inactive_values')


class StateNormalizer(Node):
    def __init__(self):
        super().__init__('agt_driver_state_normalizer')
        self.declare_parameter('config_file', '')
        path = self.get_parameter('config_file').value
        self.config = yaml.safe_load(Path(path).read_text()) if path else {}
        self.cfg = self.config.get('base_state', {})
        self.pub = self.create_publisher(BaseState, self.cfg.get('output_topic','/agt/base/state'),10)
        self.last = None; self.rx = 0.0
        self.evidence={}
        self.required_sources=self.cfg.get('required_freshness_sources',[])
        if not isinstance(self.required_sources,list): raise ValueError('required_freshness_sources must be a list')
        self.verified = self.cfg.get('field_verified') is True
        if self.verified:
            for key in ('robot_profile','input_topic','message_type','stamp_field','autonomous',
                        'emergency_stop','manual_override','fault_active'):
                if not self.cfg.get(key): raise ValueError('base_state.'+key+' required')
            for key in ('autonomous','emergency_stop','manual_override','fault_active'):
                validate_mapping(self.cfg[key])
            if self.cfg['input_topic'] == self.cfg.get('output_topic','/agt/base/state'):
                raise ValueError('state loop rejected')
            cls = get_message(self.cfg['message_type'])
            self.create_subscription(cls,self.cfg['input_topic'],self.on_message,10)
            seen=set()
            for source in self.required_sources:
                name=source.get('id')
                if not name or name in seen or not source.get('input_topic') or not source.get('message_type'):
                    raise ValueError('unique explicit freshness source id/topic/type required')
                seen.add(name)
                if source['input_topic']==self.cfg.get('output_topic','/agt/base/state'):
                    raise ValueError('freshness state loop rejected')
                limit=float(source.get('timeout_sec',.3))
                if not math.isfinite(limit) or limit<=0: raise ValueError('positive freshness timeout required')
                self.create_subscription(get_message(source['message_type']),source['input_topic'],
                    lambda msg,spec=source:self.on_evidence(msg,spec),10)
        else:
            self.get_logger().warning('unverified state mapping: publishing invalid BaseState only')
        self.timeout=float(self.cfg.get('timeout_sec',.3))
        if not math.isfinite(self.timeout) or self.timeout<=0: raise ValueError('positive timeout required')
        self.create_timer(.05,self.publish)

    def on_message(self,message):
        self.last=message; self.rx=time.monotonic()

    def on_evidence(self,message,spec):
        try:
            stamp=field(message,spec['stamp_field']) if spec.get('stamp_field') else message
            source_ns=int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
            old=self.evidence.get(spec['id'])
            if source_ns<=0 or (old and source_ns<=old[0]): return
            self.evidence[spec['id']]=(source_ns,time.monotonic())
        except (ValueError,TypeError,AttributeError,KeyError):
            self.evidence.pop(spec['id'],None)

    def evidence_fresh(self,primary_ns):
        for spec in self.required_sources:
            entry=self.evidence.get(spec['id'])
            if entry is None: return False
            limit=float(spec.get('timeout_sec',.3))
            source_age=(self.get_clock().now().nanoseconds-entry[0])*1e-9
            if not -.05<=source_age<=limit or time.monotonic()-entry[1]>limit: return False
            if spec.get('require_primary_at_or_after',False) and primary_ns<entry[0]: return False
        return True

    def mapped(self,key):
        return self.mapping_value(self.cfg[key])

    def mapping_value(self,spec):
        if 'any_of' in spec:
            # Evaluate every leaf; short-circuiting would hide an invalid flag.
            values=[self.mapping_value(item) for item in spec['any_of']]
            return any(values)
        value=field(self.last,spec['field'])
        if spec.get('predicate') == 'nonzero':
            if not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('finite numeric flag required')
            return value != 0
        if value in spec['active_values']: return True
        if value in spec['inactive_values']: return False
        raise ValueError('unrecognized '+spec['field']+' value')

    def publish(self):
        out=BaseState(); out.stamp=self.get_clock().now().to_msg()
        out.robot_profile=str(self.cfg.get('robot_profile',''))
        out.reason='state_mapping_not_verified'
        if self.verified and self.last is not None:
            try:
                source=field(self.last,self.cfg['stamp_field'])
                source_ns=int(source.sec)*1_000_000_000+int(source.nanosec)
                age=(self.get_clock().now().nanoseconds-source_ns)*1e-9
                out.stamp=source
                out.source_valid=(source_ns>0 and -.05<=age<=self.timeout and
                                  time.monotonic()-self.rx<=self.timeout and self.evidence_fresh(source_ns))
                out.emergency_stop=self.mapped('emergency_stop')
                out.manual_override=self.mapped('manual_override')
                out.fault_active=self.mapped('fault_active')
                out.drive_permitted=(out.source_valid and self.mapped('autonomous') and
                                     not out.emergency_stop and not out.manual_override and not out.fault_active)
                for name,path in self.cfg.get('measured_velocity_fields',{}).items():
                    if name not in ('linear_x','angular_z'): raise ValueError('unsupported measured velocity axis')
                    spec=path if isinstance(path,dict) else {'field':path,'scale':1.0}
                    value=float(field(self.last,spec['field']))*float(spec.get('scale',1.0))
                    if not math.isfinite(value): raise ValueError('invalid measured velocity')
                    if name=='linear_x': out.measured_velocity.linear.x=value
                    else: out.measured_velocity.angular.z=value
                out.measured_velocity_valid=(out.source_valid and
                    set(self.cfg.get('measured_velocity_fields',{}))=={'linear_x','angular_z'})
                out.reason='drive_permitted' if out.drive_permitted else 'source_stale_or_platform_interlocked'
            except (ValueError,TypeError,AttributeError,KeyError) as exc:
                out.source_valid=False; out.drive_permitted=False; out.reason='invalid_mapping:'+str(exc)
        self.pub.publish(out)


def main():
    rclpy.init(); node=StateNormalizer()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()

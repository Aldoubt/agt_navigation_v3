#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPreflight(Node):
    def __init__(self):
        super().__init__('mid360_imu_preflight')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('min_samples', 100)
        self.declare_parameter('max_static_gyro_rps', 0.10)
        self.declare_parameter('require_fastlio2_mapping_compatible', False)
        self.samples = []
        self.gyro = []
        self.start = time.monotonic()
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.on_imu, 200)

    def on_imu(self, msg: Imu):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        amag = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z)
        gmag = math.sqrt(g.x*g.x + g.y*g.y + g.z*g.z)
        if math.isfinite(amag) and math.isfinite(gmag):
            self.samples.append(amag)
            self.gyro.append(gmag)

    def done(self):
        return time.monotonic() - self.start >= float(self.get_parameter('duration_sec').value)

    def report(self):
        minimum = int(self.get_parameter('min_samples').value)
        if len(self.samples) < minimum:
            return False, {'result': 'FAIL', 'reason': f'only {len(self.samples)} IMU samples, need >= {minimum}'}
        mean = statistics.fmean(self.samples)
        median = statistics.median(self.samples)
        std = statistics.pstdev(self.samples)
        gyro_mean = statistics.fmean(self.gyro)
        gyro_max = max(self.gyro)

        # MID360 integrations may expose either g-normalized or SI acceleration.
        if 0.75 <= median <= 1.25:
            inferred = 'g'
            batch_acc_norm = 1.0
        elif 7.0 <= median <= 12.5:
            inferred = 'm/s^2'
            batch_acc_norm = 9.81
        else:
            inferred = 'ambiguous'
            batch_acc_norm = None

        max_static_gyro = float(self.get_parameter('max_static_gyro_rps').value)
        stable = gyro_mean <= max_static_gyro
        batch_ready = inferred != 'ambiguous' and stable

        # The currently pinned robotics-laboratory/fast-lio2 implementation
        # multiplies Livox linear acceleration by 10 internally. Its unmodified
        # baseline therefore expects the driver-side norm to be approximately 1g.
        fastlio2_mapping_ready = inferred == 'g' and stable
        require_mapping = bool(
            self.get_parameter('require_fastlio2_mapping_compatible').value)
        ok = batch_ready and (fastlio2_mapping_ready if require_mapping else True)

        payload = {
            'result': 'PASS' if ok else 'FAIL',
            'samples': len(self.samples),
            'mean_acc_norm': mean,
            'acc_norm_mean': mean,
            'acc_norm_median': median,
            'acc_norm_std': std,
            'gyro_norm_mean_rps': gyro_mean,
            'gyro_norm_max_rps': gyro_max,
            'inferred_acceleration_unit': inferred,
            'recommended_batch_lio_acc_norm': batch_acc_norm,
            'batch_lio_unit_ready': batch_ready,
            'pinned_fastlio2_mapping_unit_ready': fastlio2_mapping_ready,
            'require_fastlio2_mapping_compatible': require_mapping,
            'static_gyro_gate_rps': max_static_gyro,
        }
        if inferred == 'ambiguous':
            payload['reason'] = 'static acceleration norm is neither near 1g nor 9.81m/s^2; inspect driver units/orientation'
        elif not stable:
            payload['reason'] = 'robot/sensor was not sufficiently static; repeat before freezing acc_norm/noise'
        elif require_mapping and not fastlio2_mapping_ready:
            payload['reason'] = (
                'raw IMU is SI acceleration but pinned mapping Fast-LIO2 multiplies it by 10; '
                'patch/replace the upstream scaling before mapping')
        elif not fastlio2_mapping_ready:
            payload['warning'] = (
                'Batch-LIO can use acc_norm=9.81, but the pinned mapping Fast-LIO2 '
                'must be fixed before using this IMU stream')
        return ok, payload


def main(args=None):
    rclpy.init(args=args)
    node = ImuPreflight()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        ok, payload = node.report()
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        raise SystemExit(0 if ok else 2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

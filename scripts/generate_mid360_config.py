#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
from pathlib import Path


def infer_host_ip(lidar_ip: str) -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # UDP connect only asks the kernel which route/source address it would use.
        sock.connect((lidar_ip, 56000))
        host_ip = sock.getsockname()[0]
    finally:
        sock.close()
    if not host_ip or host_ip.startswith('127.'):
        raise RuntimeError(f'could not infer a non-loopback host IP for {lidar_ip}')
    return host_ip


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate Livox ROS Driver 2 MID360 JSON for AGT field hardware.')
    parser.add_argument('--lidar-ip', default='192.168.1.117')
    parser.add_argument('--host-ip', default='auto', help='Host Ethernet IP on the MID360 subnet, or auto.')
    parser.add_argument('--output', default='~/.ros/agt_mid360/MID360_config.json')
    args = parser.parse_args()

    host_ip = infer_host_ip(args.lidar_ip) if args.host_ip == 'auto' else args.host_ip
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    config = {
        'lidar_summary_info': {'lidar_type': 8},
        'MID360': {
            'lidar_net_info': {
                'cmd_data_port': 56100,
                'push_msg_port': 56200,
                'point_data_port': 56300,
                'imu_data_port': 56400,
                'log_data_port': 56500,
            },
            'host_net_info': {
                'cmd_data_ip': host_ip,
                'cmd_data_port': 56101,
                'push_msg_ip': host_ip,
                'push_msg_port': 56201,
                'point_data_ip': host_ip,
                'point_data_port': 56301,
                'imu_data_ip': host_ip,
                'imu_data_port': 56401,
                'log_data_ip': '',
                'log_data_port': 56501,
            },
        },
        'lidar_configs': [{
            'ip': args.lidar_ip,
            'pcl_data_type': 1,
            'pattern_mode': 0,
            'extrinsic_parameter': {
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0,
                'x': 0,
                'y': 0,
                'z': 0,
            },
        }],
    }
    output.write_text(json.dumps(config, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'lidar_ip': args.lidar_ip, 'host_ip': host_ip, 'config': str(output)}, indent=2))


if __name__ == '__main__':
    main()

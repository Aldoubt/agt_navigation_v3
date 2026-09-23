#!/usr/bin/env bash
set -e

# Generated from the live ROS 2 topic graph.
ros2 bag record \
  /agt/odometry/local \
  /agt/chassis/odometry \
  /parameter_events

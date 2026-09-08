"""ROS2 node wrapper for map runtime integration.

This node intentionally does not own Nav2 or localization implementation.
It exposes the runtime boundary and will later delegate to adapters.
"""

try:
    import rclpy
    from rclpy.node import Node
except ImportError:  # allow documentation/static inspection without ROS installed
    rclpy = None
    Node = object


class MapRuntimeBridgeNode(Node):
    def __init__(self):
        super().__init__('map_runtime_bridge')

        self.declare_parameter('runtime_namespace', '/agt/map/runtime')
        self.current_generation = 0

        # TODO:
        # - subscribe agt_robot_interfaces/msg/MapRuntimeState
        # - provide ApplyMapRuntime service
        # - connect Nav2 lifecycle adapter
        # - connect localization adapter

        self.get_logger().info('Map runtime bridge initialized')


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS2 environment is required')

    rclpy.init(args=args)
    node = MapRuntimeBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""ROS2 node wrapper for map runtime integration.

This node owns the ROS2 communication boundary only.
Nav2 and localization backends are accessed through adapters.
"""

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    rclpy = None
    Node = object


class MapRuntimeBridgeNode(Node):
    def __init__(self):
        super().__init__('map_runtime_bridge')

        self.declare_parameter('runtime_namespace', '/agt/map/runtime')
        self.current_generation = 0
        self.current_state = 'IDLE'

        # Runtime integration TODO:
        # - create ApplyMapRuntime service
        # - publish agt_robot_interfaces/msg/MapRuntimeState
        # - connect RuntimeApplyController
        # - connect Nav2 lifecycle adapter
        # - connect localization adapter

        self.get_logger().info('map runtime bridge initialized')

    def publish_runtime_state(self):
        """Publish runtime state.

        Placeholder until agt_robot_interfaces is wired into this package.
        """
        pass

    def apply_runtime(self, request):
        """Apply a MapPackage runtime request.

        The complete flow will validate package metadata, check generation,
        then coordinate backend adapters.
        """
        self.current_state = 'VALIDATING'
        return True


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

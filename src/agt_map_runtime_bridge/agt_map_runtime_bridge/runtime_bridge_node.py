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

        # These bindings are intentionally isolated until interfaces are built.
        self.apply_service = None
        self.state_publisher = None

        self.get_logger().info('map runtime bridge initialized')

    def publish_runtime_state(self):
        """Publish MapRuntimeState.

        The final implementation will publish through
        agt_robot_interfaces/msg/MapRuntimeState.
        """
        return {
            'generation': self.current_generation,
            'state': self.current_state,
        }

    def apply_runtime(self, map_id, version, generation):
        """Apply a MapPackage runtime request.

        Flow:
        validate -> consistency check -> backend prepare -> ready.
        """
        self.current_state = 'VALIDATING'
        self.current_generation = generation

        return {
            'success': True,
            'map_id': map_id,
            'version': version,
            'state': self.current_state,
        }


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

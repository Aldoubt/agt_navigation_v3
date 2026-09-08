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

try:
    from agt_robot_interfaces.msg import MapRuntimeState
    from agt_robot_interfaces.srv import ApplyMapRuntime
except ImportError:
    MapRuntimeState = None
    ApplyMapRuntime = None


class MapRuntimeBridgeNode(Node):
    def __init__(self):
        super().__init__('map_runtime_bridge')

        self.declare_parameter('runtime_namespace', '/agt/map/runtime')
        self.current_generation = 0
        self.current_state = 'IDLE'

        self.apply_service = None
        self.state_publisher = None

        self._setup_interfaces()
        self.get_logger().info('map runtime bridge initialized')

    def _setup_interfaces(self):
        """Create ROS2 interfaces when agt_robot_interfaces is available."""
        if ApplyMapRuntime is not None:
            self.apply_service = self.create_service(
                ApplyMapRuntime,
                '/agt/map/runtime/apply',
                self.apply_callback,
            )

        if MapRuntimeState is not None:
            self.state_publisher = self.create_publisher(
                MapRuntimeState,
                '/agt/map/runtime/state',
                10,
            )

    def set_state(self, state, generation=None):
        self.current_state = state
        if generation is not None:
            self.current_generation = generation
        return self.publish_runtime_state()

    def publish_runtime_state(self):
        if self.state_publisher is None or MapRuntimeState is None:
            return {
                'generation': self.current_generation,
                'state': self.current_state,
            }

        msg = MapRuntimeState()
        msg.generation = self.current_generation
        msg.state = self.current_state
        self.state_publisher.publish(msg)
        return msg

    def apply_callback(self, request, response):
        result = self.apply_runtime(
            request.map_id,
            request.version,
            request.generation,
        )

        response.success = result['success']
        response.reason = result.get('reason', '')
        response.generation = request.generation
        return response

    def apply_runtime(self, map_id, version, generation):
        self.set_state('VALIDATING', generation)

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

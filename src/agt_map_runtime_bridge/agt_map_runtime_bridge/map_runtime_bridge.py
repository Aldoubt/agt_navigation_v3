from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MapRuntimeBridge(Node):
    """Apply validated map generations to runtime consumers.

    This first implementation only establishes the runtime boundary. Concrete
    Nav2 lifecycle transitions and localization backend reload hooks are kept as
    adapters so they can be integrated without coupling Map Manager to either
    backend.
    """

    def __init__(self) -> None:
        super().__init__('agt_map_runtime_bridge')

        self.declare_parameter('map_status_topic', '/agt/map/status')
        self.declare_parameter('runtime_state_topic', '/agt/map/runtime/state')

        self._state_pub = self.create_publisher(
            String,
            self.get_parameter('runtime_state_topic').value,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter('map_status_topic').value,
            self._on_map_status,
            10,
        )

        self._current_generation = None

    def _on_map_status(self, msg: String) -> None:
        """Receive Map Manager state and publish runtime application state."""
        if msg.data == self._current_generation:
            return

        self._current_generation = msg.data

        state = String()
        state.data = f'accepted_map_generation:{msg.data}'
        self._state_pub.publish(state)

        self.get_logger().info(state.data)


def main(args=None):
    rclpy.init(args=args)
    node = MapRuntimeBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

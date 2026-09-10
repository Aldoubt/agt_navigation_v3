import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


class MappingBackendAdapter(Node):
    """Backend abstraction for mapping session.

    This node intentionally does not contain FAST-LIO2 logic.
    A backend plugin can be connected later.
    """

    def __init__(self):
        super().__init__('agt_mapping_backend_adapter')

        self.state = 'IDLE'

        self.status_pub = self.create_publisher(
            String,
            '/agt/mapping/backend/status',
            10)

        self.create_service(
            Trigger,
            '/agt/mapping/backend/start',
            self.start)

        self.create_service(
            Trigger,
            '/agt/mapping/backend/stop',
            self.stop)

        self.create_service(
            Trigger,
            '/agt/mapping/backend/save',
            self.save)

        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def start(self, request, response):
        self.state = 'RUNNING'
        response.success = True
        response.message = 'backend started'
        return response

    def stop(self, request, response):
        self.state = 'STOPPED'
        response.success = True
        response.message = 'backend stopped'
        return response

    def save(self, request, response):
        self.state = 'SAVING'
        response.success = True
        response.message = 'backend save requested'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MappingBackendAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

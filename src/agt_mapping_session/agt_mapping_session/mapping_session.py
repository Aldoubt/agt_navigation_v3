import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


class MappingSession(Node):
    def __init__(self):
        super().__init__('agt_mapping_session')

        self.state = 'IDLE'

        self.status_pub = self.create_publisher(
            String,
            '/agt/mapping/status',
            10)

        self.create_service(Trigger, '/agt/mapping/start', self.start)
        self.create_service(Trigger, '/agt/mapping/stop', self.stop)
        self.create_service(Trigger, '/agt/mapping/save', self.save)

        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def start(self, request, response):
        self.state = 'MAPPING'
        response.success = True
        response.message = 'mapping started'
        return response

    def stop(self, request, response):
        self.state = 'STOPPING'
        response.success = True
        response.message = 'mapping stopping'
        return response

    def save(self, request, response):
        self.state = 'SAVING'
        response.success = True
        response.message = 'save pipeline triggered'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MappingSession()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

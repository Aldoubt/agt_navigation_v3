import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


class MappingSession(Node):
    """Lifecycle controller for mapping sessions.

    This node intentionally does not own mapping backend implementation.
    It coordinates session state and exposes capability APIs for HMI or
    higher-level orchestration.
    """

    VALID_STATES = [
        'IDLE',
        'STARTING',
        'MAPPING',
        'STOPPING',
        'SAVING',
        'READY',
        'ERROR',
    ]

    def __init__(self):
        super().__init__('agt_mapping_session')

        self.state = 'IDLE'
        self.session_id = None
        self.last_result = 'none'

        self.status_pub = self.create_publisher(
            String,
            '/agt/mapping/status',
            10)

        self.result_pub = self.create_publisher(
            String,
            '/agt/mapping/result',
            10)

        self.create_service(
            Trigger,
            '/agt/mapping/start',
            self.start)

        self.create_service(
            Trigger,
            '/agt/mapping/stop',
            self.stop)

        self.create_service(
            Trigger,
            '/agt/mapping/save',
            self.save)

        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = (
            f'state={self.state};'
            f'session={self.session_id};'
            f'result={self.last_result}'
        )
        self.status_pub.publish(msg)

    def publish_result(self, text):
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)

    def start(self, request, response):
        if self.state not in ['IDLE', 'READY']:
            response.success = False
            response.message = 'mapping session busy'
            return response

        self.state = 'STARTING'
        self.session_id = self.get_clock().now().nanoseconds
        self.last_result = 'started'

        # Backend adapter connection will be added in next stage.
        self.state = 'MAPPING'

        response.success = True
        response.message = 'mapping session started'
        return response

    def stop(self, request, response):
        if self.state != 'MAPPING':
            response.success = False
            response.message = 'no active mapping session'
            return response

        self.state = 'STOPPING'
        self.last_result = 'stopped'

        response.success = True
        response.message = 'mapping session stopped'
        return response

    def save(self, request, response):
        if self.state not in ['STOPPING', 'MAPPING']:
            response.success = False
            response.message = 'invalid state for save'
            return response

        self.state = 'SAVING'

        # Save backend / converter pipeline will be connected later.
        self.last_result = 'save_requested'
        self.publish_result('save_requested')

        response.success = True
        response.message = 'save pipeline requested'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MappingSession()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

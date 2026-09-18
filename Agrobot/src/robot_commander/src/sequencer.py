import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class Sequencer(Node):

    def __init__(self):
        super().__init__('sequencer')
        self._pub = self.create_publisher(Bool, '/agrobot/operator_ready', 10)
        self.create_subscription(Bool, '/agrobot/start_pick', self.start_pick_callback, 10)
        self.get_logger().info('Sequencer node initialized, waiting for /start_pick')

    def start_pick_callback(self, msg: Bool):
        self._pub.publish(msg)
        state = 'ready' if msg.data else 'not ready'
        self.get_logger().info(f'Operator is {state}, published to /operator_ready')

if __name__ == '__main__':
    rclpy.init()
    node = Sequencer()
    rclpy.spin(node)
    rclpy.shutdown()
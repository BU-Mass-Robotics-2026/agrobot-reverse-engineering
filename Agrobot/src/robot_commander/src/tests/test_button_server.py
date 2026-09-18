import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from robot_interfaces.srv import SetIndicator


class TestButtonServer(Node):
    def __init__(self):
        super().__init__('test_button_server')
        self._count = 0
        self._state = False
        self.srv = self.create_service(Trigger, '/agrobot/test_button', self.button_callback)
        self._indicator_cli = self.create_client(SetIndicator, '/agrobot/plc/set_indicator')
        self.get_logger().info(
            'test_button_server ready — waiting for calls on /agrobot/test_button'
        )

    def button_callback(self, request, response):
        self._count += 1
        self._state = not self._state
        state_str = 'ON' if self._state else 'OFF'
        self.get_logger().info(f'Button pressed {self._count} times, state is now {state_str}')

        if self._indicator_cli.service_is_ready():
            req = SetIndicator.Request()
            req.indicator_id = 'test'
            req.state = self._state
            self._indicator_cli.call_async(req)
        else:
            self.get_logger().warn('set_indicator service not available, skipping light update')

        response.success = True
        response.message = f'Button state toggled to {state_str}, total presses: {self._count}'
        return response


def main():
    rclpy.init()

    node = TestButtonServer()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from robobo_ros2_interfaces.srv import SayText


class SpeechNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('speech_node')

        self.rob = rob
        self._namespace = f'/robobo/robot_{robot_name}/smartphone'

        self.create_service(SayText,
                            f'{self._namespace}/say_text',
                            self.say_text_cb)

        self.get_logger().info('SpeechNode started')

    def say_text_cb(self, request, response):
        try:
            self.rob.sayText(
                request.text,
                bool(request.wait)
            )
            response.success = True
        except Exception as e:
            self.get_logger().error(f'sayText failed: {e}')
            response.success = False
        return response
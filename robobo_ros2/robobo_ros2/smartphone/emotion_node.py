import rclpy
from rclpy.node import Node

from robobo_ros2_interfaces.srv import SetEmotion
from robobopy.utils.Emotions import Emotions


class EmotionNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('emotion_node')

        self.rob = rob
        self._namespace = f'/robobo/robot_{robot_name}/smartphone'

        self.create_service(SetEmotion,
                            f'{self._namespace}/set_emotion',
                            self.set_emotion_cb)

        self.get_logger().info('EmotionNode started')

    def set_emotion_cb(self, request, response):
        try:
            emotion = Emotions[request.emotion]
            self.rob.setEmotionTo(emotion)
            response.success = True
        except Exception as e:
            self.get_logger().error(f'setEmotion failed: {e}')
            response.success = False
        return response
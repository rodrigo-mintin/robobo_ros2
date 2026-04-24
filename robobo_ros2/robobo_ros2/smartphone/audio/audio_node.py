import rclpy
from rclpy.node import Node

from robobo_ros2_interfaces.srv import PlayNote, PlaySound
from robobopy.utils.Sounds import Sounds


class AudioNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('audio_node')

        self.rob = rob
        self._namespace = f'/robobo/robot_{robot_name}/smartphone'

        self.create_service(PlayNote,
                            f'{self._namespace}/play_note',
                            self.play_note_cb)

        self.create_service(PlaySound,
                            f'{self._namespace}/play_sound',
                            self.play_sound_cb)

        self.get_logger().info('AudioNode started')

    def play_note_cb(self, request, response):
        try:
            self.rob.playNote(
                int(request.note),
                float(request.duration),
                bool(request.wait)
            )
            response.success = True
        except Exception as e:
            self.get_logger().error(f'playNote failed: {e}')
            response.success = False
        return response

    def play_sound_cb(self, request, response):
        try:
            sound = Sounds[request.sound]
            self.rob.playSound(sound)
            response.success = True
        except Exception as e:
            self.get_logger().error(f'playSound failed: {e}')
            response.success = False
        return response
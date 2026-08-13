import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from robobo_ros2_interfaces.msg import DetectedNote


class NoiseAndNotesNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('noise_and_notes_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.noise_pub = self.create_publisher(
            Float32,
            f'{self._namespace}/ambient_noise',
            10
        )

        self.note_pub = self.create_publisher(
            DetectedNote,
            f'{self._namespace}/detected_note',
            10
        )

        self.timer = self.create_timer(0.2, self.publish_audio_sensors)

        self.get_logger().info('NoiseAndNotesNode started')

    def publish_audio_sensors(self):
        # 1. Noise Level (SPL in dB)
        try:
            noise_val = self.rob.readNoiseLevel()
            if noise_val is not None:
                msg = Float32()
                msg.data = float(noise_val)
                self.noise_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Noise read failed: {e}')

        # 2. Last Note Detected
        try:
            note_obj = self.rob.readLastNote()
            if note_obj and getattr(note_obj, 'name', None) and note_obj.name != "None":
                note_msg = DetectedNote()
                note_msg.note = str(note_obj.name)
                note_msg.duration = int(getattr(note_obj, 'duration', 0))
                self.note_pub.publish(note_msg)
        except Exception as e:
            self.get_logger().error(f'Note read failed: {e}')

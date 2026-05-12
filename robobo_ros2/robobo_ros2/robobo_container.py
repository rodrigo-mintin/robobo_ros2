import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robobopy.Robobo import Robobo

# Import your nodes
from robobo_ros2.base.robobo_base_node import RoboboBaseNode
from robobo_ros2.smartphone.imu_node import IMUNode
from robobo_ros2.smartphone.battery_node import BatteryNode
from robobo_ros2.smartphone.light_node import LightNode

from robobo_ros2.smartphone.audio.audio_node import AudioNode
from robobo_ros2.smartphone.audio.speech_node import SpeechNode
from robobo_ros2.smartphone.emotion_node import EmotionNode

from robobo_ros2.smartphone.vision.qr_node import QRNode
from robobo_ros2.smartphone.vision.aruco_node import ArucoNode

from robobo_ros2.smartphone.vision.camera_node import CameraNode


class RoboboContainer(Node):

    def __init__(self):
        super().__init__('robobo_container')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('robot_name', '0')
        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('robot_id', 0)

        # Smartphone modules (list form is cleaner)
        self.declare_parameter('modules', [
            'imu', 'brightness', 'audio', 'speech'
            'camera', 'blob', 'qr', 'aruco', 'emotion'
            ])

        self.robot_name = self.get_parameter('robot_name').value
        self.ip = self.get_parameter('ip').value
        self.robot_id = self.get_parameter('robot_id').value
        self.modules = self.get_parameter('modules').value

        # --- Namespace ---
        self._namespace = f'/robobo/robot_{self.robot_name}/base'
        self.get_logger().info(f"Namespace: {self._namespace}")

        # -------------------------
        # Create Robobo connection
        # -------------------------
        self.get_logger().info('Connecting to Robobo...')
        self.rob = Robobo(self.ip, robot_id=self.robot_id)
        self.rob.connect()

        # -------------------------
        # Create nodes
        # -------------------------
        self.nodes = []

        # Base node is ALWAYS required
        self.base_node = RoboboBaseNode(self.rob, self.robot_name)
        self.nodes.append(self.base_node)

        self.nodes.append(BatteryNode(self.rob, self.robot_name))

        if 'imu' in self.modules:
            self.nodes.append(IMUNode(self.rob, self.robot_name))
        
        if 'brightness' in self.modules:
            self.nodes.append(LightNode(self.rob, self.robot_name))

        if 'audio' in self.modules:
            self.nodes.append(AudioNode(self.rob, self.robot_name))

        if 'speech' in self.modules:
            self.nodes.append(SpeechNode(self.rob, self.robot_name))

        if 'emotion' in self.modules:
            self.nodes.append(EmotionNode(self.rob, self.robot_name))
        
        if 'qr' in self.modules:
            self.nodes.append(QRNode(self.rob, self.robot_name))
        
        if 'aruco' in self.modules:
            self.nodes.append(ArucoNode(self.rob, self.robot_name))
        
        if 'camera' in self.modules:
            self.nodes.append(CameraNode(self.rob, self.robot_name, self.ip))


def main(args=None):
    rclpy.init(args=args)
    container = RoboboContainer()
    executor = MultiThreadedExecutor()

    # Add container itself (optional, useful for logs/params)
    executor.add_node(container)

    # Add all internal nodes
    for node in container.nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    finally:
        # Cleanup
        for node in container.nodes:
            node.destroy_node()

        container.destroy_node()
        rclpy.shutdown()
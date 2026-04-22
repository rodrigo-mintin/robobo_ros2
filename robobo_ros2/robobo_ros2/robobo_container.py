import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robobopy.Robobo import Robobo

# Import your nodes
from robobo_ros2.base.robobo_base_node import RoboboBaseNode
from robobo_ros2.smartphone.orientation.orientation_node import OrientationNode
#from robobo_ros2.smartphone.camera.camera_node import CameraNode
# from robobo_ros2.smartphone.qr.qr_node import QRNode
# from robobo_ros2.smartphone.aruco.aruco_node import ArucoNode
# from robobo_ros2.smartphone.voice.voice_node import VoiceNode


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
        self.declare_parameter('modules', ['camera'])

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

        # Optional modules
        if 'camera' in self.modules:
            self.get_logger().info('Loading CameraNode')
            self.nodes.append(CameraNode(self.rob, self.robot_name))

        if 'orientation' in self.modules:
            self.get_logger().info('Loading OrientationNode')
            self.nodes.append(OrientationNode(self.rob, self.robot_name))


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
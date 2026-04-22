import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import math
import time


class OrientationNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('orientation_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.publisher = self.create_publisher(
            Imu,
            f'{self._namespace}/orientation',
            10
        )

        # 10 Hz is enough
        self.timer = self.create_timer(0.1, self.publish_orientation)

        self.get_logger().info('OrientationNode started')

    def publish_orientation(self):
        try:
            # -------------------------
            # Get orientation from robot
            # -------------------------
            # Adjust this depending on robobopy API
            orientation = self.rob.readOrientation()

            # Example expected:
            # {
            #   'yaw': ...,
            #   'pitch': ...,
            #   'roll': ...,
            #   'timestamp': ...
            # }

            if not isinstance(orientation, dict):
                return

            yaw = orientation.get('yaw', 0.0)
            pitch = orientation.get('pitch', 0.0)
            roll = orientation.get('roll', 0.0)

            # -------------------------
            # Convert to quaternion
            # -------------------------
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

            # -------------------------
            # Fill ROS message
            # -------------------------
            msg = Imu()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Orientation read failed: {e}')

    # -------------------------
    # Euler → Quaternion
    # -------------------------
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Assuming radians (convert if needed)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw
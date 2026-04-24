import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import math


class IMUNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('imu_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.publisher = self.create_publisher(
            Imu,
            f'{self._namespace}/imu',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_imu)

        self.get_logger().info('IMU node started')

    def publish_imu(self):
        try:
            # -------------------------
            # Orientation
            # -------------------------
            orientation = self.rob.readOrientationSensor()

            yaw = orientation.yaw
            pitch = orientation.pitch
            roll = orientation.roll

            # Convert degrees → radians (very likely needed)
            yaw = math.radians(yaw)
            pitch = math.radians(pitch)
            roll = math.radians(roll)

            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

            # -------------------------
            # Acceleration
            # -------------------------
            acc = self.rob.readAccelerationSensor()

            # Assuming object has attributes x, y, z
            ax = acc.x
            ay = acc.y
            az = acc.z

            # -------------------------
            # Build message
            # -------------------------
            msg = Imu()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            # Orientation
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            # Linear acceleration
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)

            # -------------------------
            # Covariances
            # -------------------------

            # Unknown → set to -1
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            # No gyro data
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'IMU read failed: {e}')

    def euler_to_quaternion(self, roll, pitch, yaw):
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
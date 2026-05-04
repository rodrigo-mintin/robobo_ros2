import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class LightNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('light_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.publisher = self.create_publisher(
            Float32,
            f'{self._namespace}/illuminance',
            10
        )

        # Light can change moderately fast
        self.timer = self.create_timer(0.5, self.publish_light)

        self.get_logger().info('LightNode started')

    def publish_light(self):
        try:
            value = self.rob.readBrightnessSensor()  # or readLightSensor()

            msg = Float32()
            msg.data = float(value)  # IMPORTANT (avoid ROS2 crash)

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Light read failed: {e}')
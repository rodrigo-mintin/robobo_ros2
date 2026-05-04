import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class BatteryNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('battery_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.publisher = self.create_publisher(
            Float32,
            f'{self._namespace}/battery',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_battery)

        self.get_logger().info('BatteryNode started')

    def publish_battery(self):
        try:
            value = self.rob.readBatteryLevel('phone')

            msg = Float32()
            msg.data = float(value)   # IMPORTANT

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Battery read failed: {e}')
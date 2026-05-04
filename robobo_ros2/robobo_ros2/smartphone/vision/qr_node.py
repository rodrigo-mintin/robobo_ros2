import rclpy
from rclpy.node import Node

from robobo_ros2_interfaces.msg import QR
from geometry_msgs.msg import Point


class QRNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('qr_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.publisher = self.create_publisher(
            QR,
            f'{self._namespace}/qr',
            10
        )

        self.rob.startQrTracking()

        self.timer = self.create_timer(0.2, self.publish_qr)

        self.get_logger().info('QRNode started')

    def publish_qr(self):
        try:
            qr = self.rob.readQR()

            if qr.id == "None":
                return

            msg = QR()

            # -------------------------
            # Direct attribute access (correct SDK usage)
            # -------------------------
            msg.id = int(qr.id)
            msg.x = float(qr.x)
            msg.y = float(qr.y)

            msg.distance = float(qr.distance)

            # -------------------------
            # Points (dicts still need conversion)
            # -------------------------
            msg.p1 = self._to_point(qr.p1)
            msg.p2 = self._to_point(qr.p2)
            msg.p3 = self._to_point(qr.p3)

            msg.stamp = self.get_clock().now().to_msg()

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'QR read failed: {e}')

    def _to_point(self, p):
        pt = Point()
        pt.x = float(p['x'])
        pt.y = float(p['y'])
        pt.z = 0.0
        return pt
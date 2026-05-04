import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from robobo_ros2_interfaces.msg import ArucoArray, ArucoTag


class ArucoNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('aruco_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.publisher = self.create_publisher(
            ArucoArray,
            f'{self._namespace}/aruco',
            10
        )

        self.rob.startArUcoTagDetection()

        self.timer = self.create_timer(0.2, self.publish_aruco)

        self.get_logger().info('ArucoNode started')

    def publish_aruco(self):
        try:
            tags = self.rob.readArucoTags()

            msg = ArucoArray()
            msg.tags = []

            if not tags:
                self.publisher.publish(msg)
                return

            for t in tags:

                tag_msg = ArucoTag()

                # -------------------------
                # ID
                # -------------------------
                tag_msg.id = int(t.id)

                # -------------------------
                # Corners
                # -------------------------
                tag_msg.cor1 = self._pt(t.cor1)
                tag_msg.cor2 = self._pt(t.cor2)
                tag_msg.cor3 = self._pt(t.cor3)
                tag_msg.cor4 = self._pt(t.cor4)

                # -------------------------
                # Pose vectors
                # -------------------------
                tag_msg.translation = self._vec(t.tvecs)
                tag_msg.rotation = self._vec(t.rvecs)

                # -------------------------
                # Timestamp
                # -------------------------
                tag_msg.stamp = self.get_clock().now().to_msg()

                msg.tags.append(tag_msg)

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Aruco read failed: {e}')

    # -------------------------
    # Helpers
    # -------------------------
    def _pt(self, d):
        p = Point()
        p.x = float(d['x'])
        p.y = float(d['y'])
        p.z = 0.0
        return p

    def _vec(self, d):
        p = Point()
        p.x = float(d['x'])
        p.y = float(d['y'])
        p.z = float(d['z'])
        return p
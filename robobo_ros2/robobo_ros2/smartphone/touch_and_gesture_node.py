import rclpy
from rclpy.node import Node

from robobo_ros2_interfaces.msg import Tap, Fling
from robobo_ros2_interfaces.srv import ResetTapSensor, ResetFlingSensor


class TouchAndGestureNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('touch_and_gesture_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.tap_pub = self.create_publisher(
            Tap,
            f'{self._namespace}/tap',
            10
        )

        self.fling_pub = self.create_publisher(
            Fling,
            f'{self._namespace}/fling',
            10
        )

        self.create_service(
            ResetTapSensor,
            f'{self._namespace}/reset_tap_sensor',
            self.reset_tap_cb
        )

        self.create_service(
            ResetFlingSensor,
            f'{self._namespace}/reset_fling_sensor',
            self.reset_fling_cb
        )

        self.timer = self.create_timer(0.1, self.publish_touch_sensors)

        self.get_logger().info('TouchAndGestureNode started')

    def publish_touch_sensors(self):
        # 1. Tap Sensor
        try:
            tap_obj = self.rob.readTapSensor()
            if tap_obj and (tap_obj.x != 0 or tap_obj.y != 0):
                msg = Tap()
                msg.x = int(tap_obj.x)
                msg.y = int(tap_obj.y)
                msg.zone = str(tap_obj.zone) if tap_obj.zone else ""
                self.tap_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Tap read failed: {e}')

        # 2. Fling Sensor
        try:
            distance = self.rob.readFlingDistance()
            if distance and distance > 0:
                msg = Fling()
                msg.angle = int(self.rob.readFlingAngle() or 0)
                msg.distance = int(distance)
                msg.time = int(self.rob.readFlingTime() or 0)
                self.fling_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Fling read failed: {e}')

    def reset_tap_cb(self, request, response):
        try:
            self.rob.resetTapSensor()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'resetTapSensor failed: {e}')
            response.success = False
        return response

    def reset_fling_cb(self, request, response):
        try:
            self.rob.resetFlingSensor()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'resetFlingSensor failed: {e}')
            response.success = False
        return response

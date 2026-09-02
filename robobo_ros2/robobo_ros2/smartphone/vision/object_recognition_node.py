import rclpy
from rclpy.node import Node

from robobo_ros2_interfaces.msg import DetectedObject
from robobo_ros2_interfaces.srv import StartObjectRecognition, StopObjectRecognition


class ObjectRecognitionNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('object_recognition_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        self.publisher = self.create_publisher(
            DetectedObject,
            f'{self._namespace}/detected_object',
            10
        )

        self.create_service(
            StartObjectRecognition,
            f'{self._namespace}/start_object_recognition',
            self.start_recognition_cb
        )

        self.create_service(
            StopObjectRecognition,
            f'{self._namespace}/stop_object_recognition',
            self.stop_recognition_cb
        )

        # Auto-start object recognition
        self.rob.startObjectRecognition()

        self.timer = self.create_timer(0.2, self.publish_object)

        self.get_logger().info('ObjectRecognitionNode started')

    def publish_object(self):
        try:
            obj = self.rob.readDetectedObject()

            if not obj or not getattr(obj, 'label', None):
                return

            msg = DetectedObject()
            msg.x = int(obj.x)
            msg.y = int(obj.y)
            msg.width = int(obj.width)
            msg.height = int(obj.height)
            msg.confidence = float(obj.confidence)
            msg.label = str(obj.label)
            msg.timestamp = int(getattr(obj, 'timeStamp', 0))

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Object recognition read failed: {e}')

    def start_recognition_cb(self, request, response):
        try:
            self.rob.startObjectRecognition()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'startObjectRecognition failed: {e}')
            response.success = False
        return response

    def stop_recognition_cb(self, request, response):
        try:
            self.rob.stopObjectRecognition()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'stopObjectRecognition failed: {e}')
            response.success = False
        return response

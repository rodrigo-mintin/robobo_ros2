import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
import time

from robobopy_videostream.RoboboVideo import RoboboVideo
from robobo_ros2_interfaces.srv import SetCamera, StartCamera, StopCamera


class CameraNode(Node):

    def __init__(self, rob, robot_name, ip):
        super().__init__('camera_node')

        self.rob = rob
        self.robot_name = robot_name
        self.ip = ip

        self._namespace = f'/robobo/robot_{robot_name}/smartphone'

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            f'{self._namespace}/camera/image_raw',
            10
        )

        # Services for Camera Control
        self.create_service(
            SetCamera,
            f'{self._namespace}/set_camera',
            self.set_camera_cb
        )

        self.create_service(
            StartCamera,
            f'{self._namespace}/start_camera',
            self.start_camera_cb
        )

        self.create_service(
            StopCamera,
            f'{self._namespace}/stop_camera',
            self.stop_camera_cb
        )

        self.video = RoboboVideo(self.ip)

        self.running = False
        self.thread = None

        # Delay startup until ROS is spinning
        self.start_timer = self.create_timer(1.0, self._start_stream_once)

        self.get_logger().info('CameraNode initialized')

    def set_camera_cb(self, request, response):
        try:
            cam = request.camera.lower().strip()
            if cam in ('front', 'frontal'):
                self.rob.setFrontCamera()
                self.get_logger().info('Switched to front camera')
                response.success = True
            elif cam in ('back', 'rear'):
                self.rob.setBackCamera()
                self.get_logger().info('Switched to back camera')
                response.success = True
            else:
                self.get_logger().error(f'Unknown camera position: {request.camera}')
                response.success = False
        except Exception as e:
            self.get_logger().error(f'setCamera failed: {e}')
            response.success = False
        return response

    def start_camera_cb(self, request, response):
        try:
            self.rob.startCamera()
            if not self.running:
                self.running = True
                self.thread = threading.Thread(target=self._run_stream, daemon=True)
                self.thread.start()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'startCamera failed: {e}')
            response.success = False
        return response

    def stop_camera_cb(self, request, response):
        try:
            self.running = False
            self.rob.stopCamera()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'stopCamera failed: {e}')
            response.success = False
        return response

    def _start_stream_once(self):
        self.start_timer.cancel()
        self.get_logger().info('Starting camera stream...')
        self.running = True
        self.thread = threading.Thread(target=self._run_stream, daemon=True)
        self.thread.start()

    def _run_stream(self):
        try:
            self.rob.startStream()
            self.video.connect()
            self.get_logger().info('Video stream connected')

            while rclpy.ok() and self.running:
                frame = self.video.getImage()

                if frame is None:
                    continue

                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_link"

                self.publisher.publish(msg)
                time.sleep(0.01)

        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f'Camera stream failed: {e}')

    def destroy_node(self):
        self.running = False

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        try:
            self.video.disconnect()
        except Exception:
            pass

        super().destroy_node()
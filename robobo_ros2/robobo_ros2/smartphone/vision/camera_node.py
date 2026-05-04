import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
import time

from robobopy_videostream.RoboboVideo import RoboboVideo


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

        self.video = RoboboVideo(self.ip)

        self.running = False
        self.thread = None

        # ✅ Delay startup until ROS is spinning
        self.start_timer = self.create_timer(1.0, self._start_stream_once)

        self.get_logger().info('CameraNode initialized')

    # =====================================================
    # DELAYED START (CRITICAL FIX)
    # =====================================================
    def _start_stream_once(self):
        self.start_timer.cancel()

        self.get_logger().info('Starting camera stream...')

        self.running = True
        self.thread = threading.Thread(target=self._run_stream, daemon=True)
        self.thread.start()

    # =====================================================
    # STREAM THREAD
    # =====================================================
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

    # =====================================================
    # CLEAN SHUTDOWN
    # =====================================================
    def destroy_node(self):
        self.running = False

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        try:
            self.video.disconnect()
        except Exception:
            pass

        super().destroy_node()
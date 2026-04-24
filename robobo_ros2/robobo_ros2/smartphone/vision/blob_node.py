import rclpy
from rclpy.node import Node

from robobo_ros2_interfaces.msg import Blob, BlobArray
from robobo_ros2_interfaces.srv import SetActiveColorBlobs, ResetColorBlobs


class ColorBlobNode(Node):

    def __init__(self, rob, robot_name):
        super().__init__('color_blob_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/smartphone'

        # -------------------------
        # Publisher
        # -------------------------
        self.publisher = self.create_publisher(
            BlobArray,
            f'{self._namespace}/color_blobs',
            10
        )

        # -------------------------
        # Services (CONTROL PLANE)
        # -------------------------
        self.create_service(
            SetActiveColorBlobs,
            f'{self._namespace}/set_active_color_blobs',
            self.set_active_cb
        )

        self.create_service(
            ResetColorBlobs,
            f'{self._namespace}/reset_color_blobs',
            self.reset_cb
        )

        # Default behavior (SDK default: green only)
        self.rob.setActiveBlobs(False, True, False, False)

        # -------------------------
        # Sensor loop
        # -------------------------
        self.timer = self.create_timer(0.2, self.publish_blobs)

        self.get_logger().info('ColorBlobNode started')

    # =====================================================
    # SENSOR PUBLISHING
    # =====================================================
    def publish_blobs(self):
        try:
            blobs_dict = self.rob.readAllColorBlobs()

            msg = BlobArray()
            msg.blobs = []

            if blobs_dict:
                for _, blob in blobs_dict.items():

                    b = Blob()

                    b.color = str(blob.color)
                    b.x = float(blob.posx)
                    b.y = float(blob.posy)
                    b.size = float(blob.size)

                    b.frame_timestamp = int(blob.frame_timestamp)
                    b.status_timestamp = int(blob.status_timestamp)

                    msg.blobs.append(b)

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'ColorBlob read failed: {e}')

    # =====================================================
    # CONTROL: setActiveBlobs
    # =====================================================
    def set_active_cb(self, request, response):
        try:
            self.rob.setActiveBlobs(
                bool(request.red),
                bool(request.green),
                bool(request.blue),
                bool(request.custom)
            )

            response.success = True

        except Exception as e:
            self.get_logger().error(f'setActiveBlobs failed: {e}')
            response.success = False

        return response

    # =====================================================
    # CONTROL: reset
    # =====================================================
    def reset_cb(self, request, response):
        try:
            self.rob.resetColorBlobs()
            response.success = True

        except Exception as e:
            self.get_logger().error(f'resetColorBlobs failed: {e}')
            response.success = False

        return response
import math
import time
from unittest.mock import MagicMock
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, CameraInfo, Image

from robobo_ros2.base.robobo_base_node import RoboboBaseNode
from robobo_ros2.smartphone.vision.camera_node import CameraNode
from robobo_ros2.smartphone.imu_node import IMUNode
from robobo_ros2.smartphone.battery_node import BatteryNode
from robobo_ros2.smartphone.emotion_node import EmotionNode
from robobo_ros2.smartphone.vision.object_recognition_node import ObjectRecognitionNode
from robobo_ros2.smartphone.touch_and_gesture_node import TouchAndGestureNode


def setup_module():
    if not rclpy.ok():
        rclpy.init()


def teardown_module():
    if rclpy.ok():
        rclpy.shutdown()


def test_base_node_initialization_and_params():
    mock_rob = MagicMock()
    node = RoboboBaseNode(mock_rob, 'test_bot')

    assert node._namespace == '/robobo/robot_test_bot/base'
    assert node.cmd_vel_sub.topic_name == '/robobo/robot_test_bot/base/cmd_vel'
    assert node.odom_pub.topic_name == '/robobo/robot_test_bot/base/odom'
    assert len(node.ir_range_pubs) == 8
    assert 'frontc' in node.ir_range_pubs

    node.destroy_node()


def test_differential_kinematics_straight():
    mock_rob = MagicMock()
    node = RoboboBaseNode(mock_rob, 'test_bot')

    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = 0.0

    node.cmd_vel_callback(cmd)

    mock_rob.moveWheels.assert_called_once()
    right_speed, left_speed = mock_rob.moveWheels.call_args[0]
    assert right_speed == left_speed
    assert right_speed > 0

    node.destroy_node()


def test_differential_kinematics_turn():
    mock_rob = MagicMock()
    node = RoboboBaseNode(mock_rob, 'test_bot')

    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 1.0

    node.cmd_vel_callback(cmd)

    mock_rob.moveWheels.assert_called_once()
    right_speed, left_speed = mock_rob.moveWheels.call_args[0]
    assert right_speed == -left_speed

    node.destroy_node()


def test_cmd_vel_watchdog_timeout():
    mock_rob = MagicMock()
    node = RoboboBaseNode(mock_rob, 'test_bot')
    node.cmd_vel_timeout = 0.05

    cmd = Twist()
    cmd.linear.x = 0.1
    node.cmd_vel_callback(cmd)

    time.sleep(0.1)
    node.read_sensors()

    # Wheel stop (0,0) should be called by watchdog
    mock_rob.moveWheels.assert_called_with(0, 0)

    node.destroy_node()


def test_odometry_calculation():
    mock_rob = MagicMock()
    node = RoboboBaseNode(mock_rob, 'test_bot')

    # Mock sensor values
    mock_rob.readAllIRSensor.return_value = {0: 10, 1: 10, 2: 10, 3: 10, 4: 10, 5: 10, 6: 10, 7: 10}
    mock_rob.readBatteryLevel.return_value = 100
    mock_rob.readPanPosition.return_value = 0
    mock_rob.readTiltPosition.return_value = 0

    # First cycle sets initial encoders
    mock_rob.readWheelPosition.side_effect = lambda w: 0.0
    mock_rob.readWheelSpeed.return_value = 0
    node.read_sensors()

    assert node.last_wheel_l_pos == 0.0
    assert node.last_wheel_r_pos == 0.0

    # Second cycle with wheel movement (180 degrees)
    mock_rob.readWheelPosition.side_effect = lambda w: 180.0
    time.sleep(0.05)
    node.read_sensors()

    assert node.pose_x > 0.0

    node.destroy_node()


def test_camera_node_init():
    mock_rob = MagicMock()
    cam_node = CameraNode(mock_rob, 'test_bot', '127.0.0.1')

    assert cam_node.publisher.topic_name == '/robobo/robot_test_bot/smartphone/camera/image_raw'
    assert cam_node.camera_info_pub.topic_name == '/robobo/robot_test_bot/smartphone/camera/camera_info'

    cam_node.destroy_node()

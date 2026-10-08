import rclpy
from unittest.mock import MagicMock
from robobo_ros2.sample_demo import RoboboDemo, parse_cli_args


def setup_module():
    if not rclpy.ok():
        rclpy.init()


def teardown_module():
    if rclpy.ok():
        rclpy.shutdown()


def test_cli_arg_parsing_defaults():
    args, _ = parse_cli_args([])
    assert args.robot_name == '0'
    assert args.timeout == 15.0
    assert args.ip is None
    assert args.robot_id is None


def test_cli_arg_parsing_custom():
    args, _ = parse_cli_args(['--robot-name', 'test_bot', '--timeout', '5.0', '--ip', '192.168.1.10', '--robot-id', '2'])
    assert args.robot_name == 'test_bot'
    assert args.timeout == 5.0
    assert args.ip == '192.168.1.10'
    assert args.robot_id == 2


def test_sample_demo_node_init():
    demo = RoboboDemo(robot_name='test_bot', timeout=2.5)
    assert demo.robot_name == 'test_bot'
    assert demo.base_ns == '/robobo/robot_test_bot/base'
    assert demo.timeout == 2.5
    demo.destroy_node()


def test_wait_for_ready_timeout():
    demo = RoboboDemo(robot_name='nonexistent_bot', timeout=0.1)
    ready = demo.wait_for_ready(timeout_sec=0.1)
    assert ready is False
    demo.destroy_node()

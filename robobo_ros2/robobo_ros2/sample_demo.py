#!/usr/bin/env python3
"""
Sample demonstration script for Robobo robot capabilities.
This script demonstrates movement, LED control, and pan/tilt motor operations using
the existing robobo_ros2 package services and actions.

Usage:
  ros2 run robobo_ros2 sample_demo
  ros2 run robobo_ros2 sample_demo --ros-args -p ip:=127.0.0.1
  ros2 launch robobo_ros2 sample_demo.launch.py ip:=127.0.0.1
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor

# Import required service and action types
from robobo_ros2_interfaces.srv import SetLed, MoveWheelsTime as MoveWheelsTimeSrv
from robobo_ros2_interfaces.action import (
    MovePan,
    MoveTilt,
    MoveWheelsTime as MoveWheelsTimeAction,
)


class RoboboDemo(Node):
    def __init__(self):
        super().__init__('robobo_demo')

        # Declare parameters for connection configuration
        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('robot_name', '0', ParameterDescriptor(dynamic_typing=True))

        self.ip = str(self.get_parameter('ip').value)
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.robot_name = str(self.get_parameter('robot_name').value)

        self.base_ns = f'/robobo/robot_{self.robot_name}/base'
        self.get_logger().info(
            f"Using IP: {self.ip}, Robot ID: {self.robot_id}, Robot Name: {self.robot_name}"
        )
        self.get_logger().info(f"Connecting to base namespace: {self.base_ns}")

        # Create service clients
        self.set_led_client = self.create_client(
            SetLed, f'{self.base_ns}/set_led'
        )
        self.move_wheels_time_srv_client = self.create_client(
            MoveWheelsTimeSrv, f'{self.base_ns}/move_wheels_time'
        )

        # Create action clients
        self.move_pan_action_client = ActionClient(
            self, MovePan, f'{self.base_ns}/move_pan'
        )
        self.move_tilt_action_client = ActionClient(
            self, MoveTilt, f'{self.base_ns}/move_tilt'
        )
        self.move_wheels_time_action_client = ActionClient(
            self, MoveWheelsTimeAction, f'{self.base_ns}/move_wheels_time'
        )

    def wait_for_ready(self, timeout_sec=20.0):
        """Wait for required services and action servers to become available."""
        self.get_logger().info(
            f"Waiting up to {timeout_sec}s for services and action servers..."
        )
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            led_ready = self.set_led_client.wait_for_service(timeout_sec=1.0)
            wheels_ready = self.move_wheels_time_action_client.wait_for_server(
                timeout_sec=1.0
            )
            pan_ready = self.move_pan_action_client.wait_for_server(timeout_sec=1.0)
            tilt_ready = self.move_tilt_action_client.wait_for_server(timeout_sec=1.0)

            if led_ready and wheels_ready and pan_ready and tilt_ready:
                self.get_logger().info("All services and action servers are ready!")
                return True

            self.get_logger().info("Still waiting for services and action servers...")

        self.get_logger().error("Timed out waiting for services and action servers to become ready.")
        return False

    def set_led(self, led='All', color='RED'):
        """Call SetLed service synchronously."""
        self.get_logger().info(f"Setting LED '{led}' to {color}...")
        req = SetLed.Request()
        req.led = led
        req.color = color

        future = self.set_led_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            try:
                response = future.result()
                if response and response.success:
                    self.get_logger().info(f"LED set successfully: {response.message}")
                    return True
                else:
                    msg = response.message if response else "Empty response"
                    self.get_logger().error(f"Failed to set LED: {msg}")
            except Exception as e:
                self.get_logger().error(f"Error reading set_led response: {e}")
        else:
            self.get_logger().error("Call to set_led service timed out")
        return False

    def move_wheels(self, right_speed, left_speed, duration):
        """Send wheel movement action goal synchronously."""
        self.get_logger().info(
            f"Moving wheels: right={right_speed}, left={left_speed} for {duration}s..."
        )
        goal_msg = MoveWheelsTimeAction.Goal()
        goal_msg.right_speed = float(right_speed)
        goal_msg.left_speed = float(left_speed)
        goal_msg.time = float(duration)

        send_goal_future = self.move_wheels_time_action_client.send_goal_async(
            goal_msg
        )
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            self.get_logger().error("Timed out sending wheel movement goal")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Wheel movement goal rejected")
            return False

        self.get_logger().info("Wheel movement accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 10.0
        )

        if not result_future.done():
            self.get_logger().error("Timed out waiting for wheel movement result")
            return False

        result = result_future.result()
        success = result.result.success if result and result.result else False
        self.get_logger().info(f"Wheel movement completed. Success: {success}")
        return success

    def move_pan(self, angle, speed=30.0):
        """Send pan motor action goal synchronously."""
        self.get_logger().info(f"Moving pan motor to {angle} degrees at speed {speed}...")
        goal_msg = MovePan.Goal()
        goal_msg.angle = float(angle)
        goal_msg.speed = float(speed)

        send_goal_future = self.move_pan_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            self.get_logger().error("Timed out sending pan goal")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Pan movement goal rejected")
            return False

        self.get_logger().info("Pan movement accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.done():
            self.get_logger().error("Timed out waiting for pan result")
            return False

        result = result_future.result()
        success = result.result.success if result and result.result else False
        self.get_logger().info(f"Pan movement completed. Success: {success}")
        return success

    def move_tilt(self, angle, speed=20.0):
        """Send tilt motor action goal synchronously."""
        self.get_logger().info(f"Moving tilt motor to {angle} degrees at speed {speed}...")
        goal_msg = MoveTilt.Goal()
        goal_msg.angle = float(angle)
        goal_msg.speed = float(speed)

        send_goal_future = self.move_tilt_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            self.get_logger().error("Timed out sending tilt goal")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Tilt movement goal rejected")
            return False

        self.get_logger().info("Tilt movement accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.done():
            self.get_logger().error("Timed out waiting for tilt result")
            return False

        result = result_future.result()
        success = result.result.success if result and result.result else False
        self.get_logger().info(f"Tilt movement completed. Success: {success}")
        return success

    def run_demo(self):
        """Run the demonstration sequence."""
        self.get_logger().info("=== Starting Robobo Demonstration ===")

        # 1. Set LED to RED
        self.set_led('All', 'RED')
        time.sleep(0.5)

        # 2. Move forward for 2 seconds
        self.move_wheels(right_speed=50.0, left_speed=50.0, duration=2.0)
        time.sleep(0.5)

        # 3. Set LED to BLUE
        self.set_led('All', 'BLUE')
        time.sleep(0.5)

        # 4. Move pan motor to 45 degrees
        self.move_pan(angle=45.0, speed=30.0)
        time.sleep(0.5)

        # 5. Move tilt motor to 30 degrees
        self.move_tilt(angle=30.0, speed=20.0)
        time.sleep(0.5)

        # 6. Reset pan and tilt to 0 / neutral
        self.move_pan(angle=0.0, speed=30.0)
        self.move_tilt(angle=5.0, speed=20.0)
        time.sleep(0.5)

        # 7. Set LED to GREEN
        self.set_led('All', 'GREEN')

        self.get_logger().info("=== Robobo Demonstration Completed Successfully! ===")


def main(args=None):
    rclpy.init(args=args)
    demo = RoboboDemo()

    try:
        if demo.wait_for_ready(timeout_sec=20.0):
            demo.run_demo()
        else:
            demo.get_logger().error(
                "Demo aborted: required services and actions are not available."
            )
            sys.exit(1)
    except KeyboardInterrupt:
        demo.get_logger().info("Demonstration stopped by user.")
    except Exception as e:
        demo.get_logger().error(f"Unexpected error during demo execution: {e}")
        sys.exit(1)
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

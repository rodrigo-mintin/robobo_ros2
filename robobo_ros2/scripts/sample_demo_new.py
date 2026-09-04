#!/usr/bin/env python3
"""
Sample demonstration script for Robobo robot capabilities in a flat environment.
This script demonstrates movement, LED control, and motor operations using
the existing robobo_ros2 package services and actions.

To run with default connection:
ros2 run robobo_ros2 sample_demo_new.py

To run with specific IP connection:
ros2 run robobo_ros2 sample_demo_new.py --ros-args -p ip:=host.docker.internal
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# Import required service and action types
from robobo_ros2_interfaces.srv import MoveWheelsTime, SetLed
from robobo_ros2_interfaces.action import MovePan, MoveTilt, MoveWheelsTime as MoveWheelsTimeAction

class RoboboDemo(Node):
    def __init__(self):
        super().__init__('robobo_demo')
        
        # Declare parameters for connection configuration
        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('robot_id', 0)
        
        self.ip = self.get_parameter('ip').value
        self.robot_id = self.get_parameter('robot_id').value
        
        self.get_logger().info(f"Using IP: {self.ip}, Robot ID: {self.robot_id}")
        
        # Create service clients for basic operations
        self.move_wheels_time_client = self.create_client(MoveWheelsTime, 'move_wheels_time')
        self.set_led_client = self.create_client(SetLed, 'set_led')
        
        # Create action clients for motor operations
        self.move_pan_action_client = ActionClient(self, MovePan, 'move_pan')
        self.move_tilt_action_client = ActionClient(self, MoveTilt, 'move_tilt')
        self.move_wheels_time_action_client = ActionClient(self, MoveWheelsTimeAction, 'move_wheels_time')
        
        # Wait for services to be available
        self.get_logger().info("Waiting for services to become available...")
        self.move_wheels_time_client.wait_for_service(timeout_sec=10.0)
        self.set_led_client.wait_for_service(timeout_sec=10.0)
        
        # Wait for actions to be available
        self.get_logger().info("Waiting for action servers to become available...")
        self.move_pan_action_client.wait_for_server(timeout_sec=10.0)
        self.move_tilt_action_client.wait_for_server(timeout_sec=10.0)
        self.move_wheels_time_action_client.wait_for_server(timeout_sec=10.0)
        
        self.get_logger().info("All services and actions are ready!")
    
    def run_demo(self):
        """Run a simple demonstration sequence"""
        self.get_logger().info("Starting Robobo demonstration...")
        
        # Set LED to red
        self.get_logger().info("Setting LED to red")
        req = SetLed.Request()
        req.color = 'red'
        future = self.set_led_client.call_async(req)
        try:
            response = future.result()
            self.get_logger().info(f"LED set successfully: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Failed to set LED: {e}")
        
        # Move forward
        self.get_logger().info("Moving forward for 2 seconds")
        goal_msg = MoveWheelsTimeAction.Goal()
        goal_msg.left_wheel_speed = 50
        goal_msg.right_wheel_speed = 50
        goal_msg.duration = 2.0
        
        future = self.move_wheels_time_action_client.send_goal_async(goal_msg)
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("Forward movement accepted")
                result_future = goal_handle.get_result_async()
                result = result_future.result()
                self.get_logger().info(f"Forward movement completed. Success: {result.result.success}")
            else:
                self.get_logger().error("Forward movement rejected")
        except Exception as e:
            self.get_logger().error(f"Error during forward movement: {e}")
        
        # Set LED to blue
        self.get_logger().info("Setting LED to blue")
        req = SetLed.Request()
        req.color = 'blue'
        future = self.set_led_client.call_async(req)
        try:
            response = future.result()
            self.get_logger().info(f"LED set successfully: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Failed to set LED: {e}")
        
        # Move pan motor
        self.get_logger().info("Moving pan motor to 45 degrees")
        goal_msg = MovePan.Goal()
        goal_msg.angle = 45.0
        goal_msg.speed = 1.0
        
        future = self.move_pan_action_client.send_goal_async(goal_msg)
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("Pan motor movement accepted")
                result_future = goal_handle.get_result_async()
                result = result_future.result()
                self.get_logger().info(f"Pan motor movement completed. Success: {result.result.success}")
            else:
                self.get_logger().error("Pan motor movement rejected")
        except Exception as e:
            self.get_logger().error(f"Error during pan motor movement: {e}")
        
        self.get_logger().info("Demonstration completed!")

def main(args=None):
    rclpy.init(args=args)
    
    # Create the demo node
    demo = RoboboDemo()
    
    try:
        # Run the demonstration
        demo.run_demo()
        
    except Exception as e:
        demo.get_logger().error(f"Error during demo execution: {e}")
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
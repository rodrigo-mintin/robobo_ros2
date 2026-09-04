#!/usr/bin/env python3
"""
Sample ROS2 Python script to demonstrate Robobo's capabilities in a flat environment with four walls.
"""

import rclpy
from rclpy.node import Node
from robobo_ros2_interfaces.srv import MoveWheelsTime, SetLed
from rclpy.action import ActionClient
from robobo_ros2_interfaces.action import MovePan, MoveTilt

class RoboboDemo(Node):
    def __init__(self):
        super().__init__('robobo_demo')
        
        # Service clients
        self.move_wheels_time_client = self.create_client(MoveWheelsTime, '/robobo/robot_0/base/move_wheels_time')
        self.set_led_client = self.create_client(SetLed, '/robobo/robot_0/base/set_led')
        
        # Action clients
        self.move_pan_action_client = ActionClient(self, MovePan, '/robobo/robot_0/base/move_pan')
        self.move_tilt_action_client = ActionClient(self, MoveTilt, '/robobo/robot_0/base/move_tilt')
        
        # Wait for services to be available
        self.get_logger().info("Waiting for services...")
        self.move_wheels_time_client.wait_for_service(timeout_sec=10.0)
        self.set_led_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Services are ready!")
        
        # Run demo once
        self.get_logger().info("Starting Robobo demonstration...")
        self.run_demo()
        
    def run_demo(self):
        """Run the complete demonstration"""
        # 1. Move forward
        self.move_forward(2000)
        
        # 2. Turn left
        self.turn_left(1000)
        
        # 3. Set LED to red
        self.set_led(255, 0, 0)
        
        # 4. Move pan motor
        self.move_pan(45.0)
        
        # 5. Move tilt motor
        self.move_tilt(30.0)
        
        # 6. Move backward
        self.move_backward(2000)
        
        self.get_logger().info("Demo completed!")

    def move_forward(self, duration_ms):
        """Move robot forward"""
        self.get_logger().info("Moving forward...")
        req = MoveWheelsTime.Request()
        req.left_wheel_speed = 30
        req.right_wheel_speed = 30
        req.time_ms = duration_ms
        
        future = self.move_wheels_time_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info("Forward movement completed")
        else:
            self.get_logger().warn("Forward movement failed")

    def move_backward(self, duration_ms):
        """Move robot backward"""
        self.get_logger().info("Moving backward...")
        req = MoveWheelsTime.Request()
        req.left_wheel_speed = -30
        req.right_wheel_speed = -30
        req.time_ms = duration_ms
        
        future = self.move_wheels_time_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info("Backward movement completed")
        else:
            self.get_logger().warn("Backward movement failed")

    def turn_left(self, duration_ms):
        """Turn robot left"""
        self.get_logger().info("Turning left...")
        req = MoveWheelsTime.Request()
        req.left_wheel_speed = -20
        req.right_wheel_speed = 20
        req.time_ms = duration_ms
        
        future = self.move_wheels_time_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info("Left turn completed")
        else:
            self.get_logger().warn("Left turn failed")

    def set_led(self, red, green, blue):
        """Set LED color"""
        self.get_logger().info(f"Setting LED to RGB({red}, {green}, {blue})")
        req = SetLed.Request()
        req.led = 0
        req.red = red
        req.green = green
        req.blue = blue
        
        future = self.set_led_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info("LED set successfully")
        else:
            self.get_logger().warn("Failed to set LED")

    def move_pan(self, angle):
        """Move pan motor"""
        self.get_logger().info(f"Moving pan to {angle} degrees")
        goal = MovePan.Goal()
        goal.angle = angle
        goal.speed = 30.0
        
        future = self.move_pan_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info("Pan movement completed")
        else:
            self.get_logger().warn("Pan movement rejected")

    def move_tilt(self, angle):
        """Move tilt motor"""
        self.get_logger().info(f"Moving tilt to {angle} degrees")
        goal = MoveTilt.Goal()
        goal.angle = angle
        goal.speed = 20.0
        
        future = self.move_tilt_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info("Tilt movement completed")
        else:
            self.get_logger().warn("Tilt movement rejected")

def main(args=None):
    rclpy.init(args=args)
    
    demo = RoboboDemo()
    
    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
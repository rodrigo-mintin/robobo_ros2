import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32, Float32

from rclpy.action import ActionServer
from robobo_ros2_interfaces.action import (
    MoveWheelsTime as MoveWheelsTimeAction,
    MovePan as MovePanAction,
    MoveTilt as MoveTiltAction
)

from robobo_ros2_interfaces.srv import SetLed
from robobo_ros2_interfaces.srv import MovePan as MovePanService, MoveTilt as MoveTiltService
from robobo_ros2_interfaces.srv import ResetWheelEncoders
from robobo_ros2_interfaces.srv import (
    MoveWheels,
    MoveWheelsDegrees,
    StopWheels,
    MoveWheelsTime as MoveWheelsTimeService
)

from rclpy.action import CancelResponse

from robobopy.Robobo import Robobo
from robobopy.utils.IR import IR
from robobopy.utils.LED import LED
from robobopy.utils.Color import Color
from robobopy.utils.Wheels import Wheels

class RoboboBaseNode(Node):
    def __init__(self, rob, robot_name):
        super().__init__('robobo_base_node')

        self.rob = rob
        self.robot_name = robot_name

        self._namespace = f'/robobo/robot_{self.robot_name}/base'

        # --- IR sensors ---
        self.ir_order = [
            IR.FrontLL, IR.FrontL, IR.FrontC, IR.FrontR, IR.FrontRR,
            IR.BackL, IR.BackC, IR.BackR
        ]

        # --- Publishers ---
        # Full array
        self.ir_pub = self.create_publisher(
            Int32MultiArray,
            f'{self._namespace}/ir',
            10
        )

        # Individual sensors
        self.ir_single_pubs = {}
        for sensor in self.ir_order:
            str_sensor = sensor.name
            topic_name = f"{self._namespace}/ir/{str_sensor.lower()}"
            self.ir_single_pubs[str_sensor.lower()] = self.create_publisher(
                Int32,
                topic_name,
                10
            )

        # Battery
        self.battery_pub = self.create_publisher(
            Int32,
            f'{self._namespace}/battery',
            10
        )

        # Pan/Tilt
        self.pan_pub = self.create_publisher(
            Int32,
            f'{self._namespace}/pan/position',
            10
        )

        self.tilt_pub = self.create_publisher(
            Int32,
            f'{self._namespace}/tilt/position',
            10
        )

        # Wheels
        # Wheel position
        self.wheel_left_pos_pub = self.create_publisher(
            Int32, f'{self._namespace}/wheel/left/position', 10)

        self.wheel_right_pos_pub = self.create_publisher(
            Int32, f'{self._namespace}/wheel/right/position', 10)

        # Wheel speed
        self.wheel_left_speed_pub = self.create_publisher(
            Int32, f'{self._namespace}/wheel/left/speed', 10)

        self.wheel_right_speed_pub = self.create_publisher(
            Int32, f'{self._namespace}/wheel/right/speed', 10)

        # --- Services ---

        self.led_service = self.create_service(
            SetLed,
            f'{self._namespace}/set_led',
            self.handle_set_led
        )

        self.move_pan_srv = self.create_service(
            MovePanService,
            f'{self._namespace}/move_pan',
            self.move_pan_callback
        )

        self.move_tilt_srv = self.create_service(
            MoveTiltService,
            f'{self._namespace}/move_tilt',
            self.move_tilt_callback
        )

        self.move_wheels_srv = self.create_service(
            MoveWheels,
            f'{self._namespace}/move_wheels',
            self.move_wheels_callback
        )

        self.stop_wheels_srv = self.create_service(
            StopWheels,
            f'{self._namespace}/stop_wheels',
            self.stop_wheels_callback
        )

        self.move_wheels_time_srv = self.create_service(
            MoveWheelsTimeService,
            f'{self._namespace}/move_wheels_time',
            self.move_wheels_time_callback
        )

        self.move_wheels_degrees_srv = self.create_service(
            MoveWheels,
            f'{self._namespace}/move_wheels_degrees',
            self.move_wheels_degrees_callback
        )

        self.reset_encoders_srv = self.create_service(
            ResetWheelEncoders,
            f'{self._namespace}/reset_wheel_encoders',
            self.reset_wheel_encoders_callback
        )

        # --- Actions ---
        self.move_wheels_time_action = ActionServer(
            self,
            MoveWheelsTimeAction,
            f'{self._namespace}/move_wheels_time',
            self.execute_move_wheels_time
        )

        self.move_pan_action = ActionServer(
            self,
            MovePanAction,
            f'{self._namespace}/move_pan',
            self.execute_move_pan
        )

        self.move_tilt_action = ActionServer(
            self,
            MoveTiltAction,
            f'{self._namespace}/move_tilt',
            self.execute_move_tilt
        )

        # --- Timer ---
        self.timer = self.create_timer(0.1, self.read_sensors)

    # =========================
    # Sensor Loop
    # =========================
    def read_sensors(self):
        if not self.rob:
            return
        try:
            # --- IR sensors ---
            ir_data = self.rob.readAllIRSensor()

            if not isinstance(ir_data, dict):
                return

            ir_msg = Int32MultiArray()
            ir_msg.data = [int(ir_data.get(sensor.value, 0)) for sensor in self.ir_order]
            self.ir_pub.publish(ir_msg)

            for sensor in self.ir_order:
                msg = Int32()
                msg.data = int(ir_data.get(sensor.value, 0))
                self.ir_single_pubs[sensor.name.lower()].publish(msg)

            # --- Battery ---
            battery_level = self.rob.readBatteryLevel('base')

            battery_msg = Int32()
            battery_msg.data = int(battery_level)
            self.battery_pub.publish(battery_msg)

            # --- Pan/Tilt ---
            pan = int(self.rob.readPanPosition())
            tilt = int(self.rob.readTiltPosition())

            msg = Int32()
            msg.data = pan
            self.pan_pub.publish(msg)

            msg = Int32()
            msg.data = tilt
            self.tilt_pub.publish(msg)

            # --- Wheels ---
            left_pos = self.rob.readWheelPosition(Wheels.L)
            right_pos = self.rob.readWheelPosition(Wheels.R)

            if isinstance(left_pos, (int, float)):
                msg = Int32()
                msg.data = int(left_pos)
                self.wheel_left_pos_pub.publish(msg)

            if isinstance(right_pos, (int, float)):
                msg = Int32()
                msg.data = int(right_pos)
                self.wheel_right_pos_pub.publish(msg)

            left_speed = self.rob.readWheelSpeed(Wheels.L)
            right_speed = self.rob.readWheelSpeed(Wheels.R)

            if isinstance(left_speed, (int, float)):
                msg = Int32()
                msg.data = int(left_speed)
                self.wheel_left_speed_pub.publish(msg)

            if isinstance(right_speed, (int, float)):
                msg = Int32()
                msg.data = int(right_speed)
                self.wheel_right_speed_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Sensor read failed: {e}")

    # =========================
    # LED Service
    # =========================
    def handle_set_led(self, request, response):
        led_enum = getattr(LED, request.led)
        color_enum = getattr(Color, request.color)
        try:
            if led_enum is None:
                response.success = False
                response.message = f"Invalid LED: {request.led}"
                return response

            if color_enum is None:
                response.success = False
                response.message = f"Invalid color: {request.color}"
                return response

            self.rob.setLedColorTo(led_enum, color_enum)

            response.success = True
            response.message = "LED set successfully"

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    # =========================
    # Pan/Tilt Service
    # =========================
    def move_pan_callback(self, request, response):
        if not self.rob:
            self.get_logger().error("Robot not connected")
            response.success = False
            return response
        try:
            angle = request.angle
            speed = request.speed if request.speed > 0 else 50
            self.rob.movePanTo(angle, speed, True)

            response.success = True

        except Exception as e:
            self.get_logger().error(f"MovePan failed: {e}")
            response.success = False
        return response

    def move_tilt_callback(self, request, response):
        if not self.rob:
            self.get_logger().error("Robot not connected")
            response.success = False
            return response

        try:
            angle = request.angle
            speed = request.speed if request.speed > 0 else 50
            self.rob.moveTiltTo(angle, speed, True)
            response.success = True

        except Exception as e:
            self.get_logger().error(f"MoveTilt failed: {e}")
            response.success = False

        return response

    # =========================
    # Wheels Service
    # =========================
    def move_wheels_callback(self, request, response):
        try:
            self.rob.moveWheels(
                request.right_speed,
                request.left_speed
            )
            response.success = True

        except Exception as e:
            self.get_logger().error(f'MoveWheels failed: {e}')
            response.success = False

        return response
    
    def move_wheels_degrees_callback(self, request, response):
        try:
            self.rob.moveWheelsByDegrees(
                Wheels[request.wheel],
                request.degrees,
                request.speed
            )
            response.success = True

        except Exception as e:
            self.get_logger().error(f'MoveWheelsDegrees failed: {e}')
            response.success = False

        return response

    def stop_wheels_callback(self, request, response):
        try:
            self.rob.stopMotors()
            response.success = True

        except Exception as e:
            self.get_logger().error(f'StopWheels failed: {e}')
            response.success = False

        return response

    def move_wheels_time_callback(self, request, response):
        try:
            self.rob.moveWheelsByTime(
                request.right_speed,
                request.left_speed,
                request.time,
                True
            )
            response.success = True

        except Exception as e:
            self.get_logger().error(f'MoveWheelsTime failed: {e}')
            response.success = False

        return response
    
    def reset_wheel_encoders_callback(self, request, response):
        try:
            self.rob.resetWheelEncoders()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Failed to reset encoders: {e}')
            response.success = False
        return response
    
    # =========================
    # Wheels Actions
    # =========================
    def _move_wheels_time(self, right_speed, left_speed, duration):
        try:
            self.rob.moveWheelsByTime(
                right_speed,
                left_speed,
                duration,
                wait=False
            )
        except Exception as e:
            self.get_logger().error(f'Time movement failed: {e}')

    async def execute_move_wheels_time(self, goal_handle):

        request = goal_handle.request

        right_speed = request.right_speed
        left_speed = request.left_speed
        duration = request.time

        try:
            # Safety: stop previous motion
            self.rob.stopMotors()
            time.sleep(0.05)

            # Start thread
            thread = threading.Thread(
                target=self._move_wheels_time_blocking,
                args=(right_speed, left_speed, duration),
                daemon=True
            )
            thread.start()

            feedback_msg = MoveWheelsTimeAction.Feedback()

            start_time = time.time()

            # Monitor loop
            while thread.is_alive():
                if goal_handle.is_cancel_requested:
                    self.rob.stopMotors()
                    goal_handle.canceled()
                    return MoveWheelsTimeAction.Result(success=False)

                elapsed = time.time() - start_time
                progress = min(elapsed / max(duration, 1e-5), 1.0)

                feedback_msg.time_elapsed = float(elapsed)
                feedback_msg.progress = float(progress)

                goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.05)

            goal_handle.succeed()
            return MoveWheelsTimeAction.Result(success=True)

        except Exception as e:
            self.get_logger().error(f'Action failed: {e}')
            goal_handle.abort()
            return MoveWheelsTimeAction.Result(success=False)
    
    # =========================
    # Pan/Tilt Actions
    # =========================
    def _move_pan(self, angle, speed):
        try:
            self.rob.movePanTo(angle, speed, False)
        except Exception as e:
            self.get_logger().error(f'Pan movement failed: {e}')
    
    def _move_tilt(self, angle, speed):
        try:
            self.rob.moveTiltTo(angle, speed, False)
        except Exception as e:
            self.get_logger().error(f'Tilt movement failed: {e}')

    def compute_progress(self, start, current, target):
        total = target - start

        if abs(total) < 1e-5:
            return 1.0

        progress = (current - start) / total
        return max(0.0, min(1.0, progress))

    async def execute_move_pan(self, goal_handle):
        request = goal_handle.request

        target_angle = float(request.angle)
        speed = float(request.speed)

        try:
            # Get starting position
            start_angle = self.rob.readPanPosition()

            thread = threading.Thread(
                target=self._move_pan_blocking,
                args=(target_angle, speed),
                daemon=True
            )
            thread.start()

            feedback_msg = MovePanAction.Feedback()

            while thread.is_alive():
                if goal_handle.is_cancel_requested:
                    with self.rob_lock:
                        self.rob.stopWheels()  # or stop pan if exists
                    goal_handle.canceled()
                    return MovePanAction.Result(success=False)

                current_angle = self.rob.readPanPosition()

                progress = self.compute_progress(start_angle, current_angle, target_angle)

                feedback_msg.current_angle = float(current_angle)
                feedback_msg.progress = float(progress)

                goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.05)

            # Final update
            current_angle = self.rob.readPanPosition()

            feedback_msg.current_angle = float(current_angle)
            feedback_msg.progress = 1.0
            goal_handle.publish_feedback(feedback_msg)

            goal_handle.succeed()
            return MovePanAction.Result(success=True)

        except Exception as e:
            self.get_logger().error(f'MovePan failed: {e}')
            goal_handle.abort()
            return MovePanAction.Result(success=False)
    
    async def execute_move_tilt(self, goal_handle):
        request = goal_handle.request

        target_angle = float(request.angle)
        speed = float(request.speed)

        try:
            start_angle = self.rob.readTiltPosition()

            thread = threading.Thread(
                target=self._move_tilt_blocking,
                args=(target_angle, speed),
                daemon=True
            )
            thread.start()

            feedback_msg = MoveTiltAction.Feedback()

            while thread.is_alive():
                if goal_handle.is_cancel_requested:
                    with self.rob_lock:
                        self.rob.stopWheels()  # replace if tilt stop exists
                    goal_handle.canceled()
                    return MoveTiltAction.Result(success=False)

                current_angle = self.rob.readTiltPosition()

                progress = self.compute_progress(start_angle, current_angle, target_angle)

                feedback_msg.current_angle = float(current_angle)
                feedback_msg.progress = float(progress)

                goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.05)

            current_angle = self.rob.readTiltPosition()

            feedback_msg.current_angle = float(current_angle)
            feedback_msg.progress = 1.0
            goal_handle.publish_feedback(feedback_msg)

            goal_handle.succeed()
            return MoveTiltAction.Result(success=True)

        except Exception as e:
            goal_handle.abort()
            return MoveTiltAction.Result(success=False)

    # =========================
    # Shutdown
    # =========================
    def destroy_node(self):
        self.get_logger().info("Disconnecting from Robobo")
        try:
            self.rob.disconnect()
        except Exception:
            pass
        super().destroy_node()
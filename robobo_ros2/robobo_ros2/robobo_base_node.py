import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32, Float32

from rclpy.action import ActionServer
from robobo_ros2_interfaces.action import (
    MoveWheelsDegrees
)

from robobo_ros2_interfaces.srv import SetLed
from robobo_ros2_interfaces.srv import SetPan, SetTilt
from robobo_ros2_interfaces.srv import (
    MoveWheels,
    StopWheels,
    MoveWheelsTime
)

from rclpy.action import CancelResponse

from robobopy.Robobo import Robobo
from robobopy.utils.IR import IR
from robobopy.utils.LED import LED
from robobopy.utils.Color import Color
from robobopy.utils.Wheels import Wheels

class RoboboBaseNode(Node):

    def __init__(self):
        # Temporary init to read parameters
        super().__init__('robobo_base_node')

        # --- Parameters ---
        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('robot_id', 0)

        ip = self.get_parameter('ip').value
        robot_id = self.get_parameter('robot_id').value

        # --- Namespace ---
        self._namespace = f'/robobo/robot_{robot_id}/base'
        self.get_logger().info(f"Namespace: {self._namespace}")

        # --- Connect to Robobo ---
        self.rob = Robobo(ip, robot_id)
        self.rob.connect()

        self.rob.moveWheelsByTime(10,10,0.5,wait=True)
        self.rob.moveWheelsByTime(-10,-10,0.5,wait=False)

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

        self.set_pan_srv = self.create_service(
            SetPan,
            f'{self._namespace}/set_pan',
            self.set_pan_callback
        )

        self.set_tilt_srv = self.create_service(
            SetTilt,
            f'{self._namespace}/set_tilt',
            self.set_tilt_callback
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
            MoveWheelsTime,
            f'{self._namespace}/move_wheels_time',
            self.move_wheels_time_callback
        )

        # --- Actions ---
        self.move_wheels_degrees_action = ActionServer(
            self,
            MoveWheelsDegrees,
            f'{self._namespace}/move_wheels_degrees',
            self.execute_move_wheels_degrees
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
    def set_pan_callback(self, request, response):
        if not self.rob:
            self.get_logger().error("Robot not connected")
            response.success = False
            return response
        try:
            angle = request.angle
            speed = request.speed if request.speed > 0 else 50

            # Non-blocking call
            self.rob.movePanTo(angle, speed, False)

            response.success = True

        except Exception as e:
            self.get_logger().error(f"SetPan failed: {e}")
            response.success = False
        return response

    def set_tilt_callback(self, request, response):
        if not self.rob:
            self.get_logger().error("Robot not connected")
            response.success = False
            return response

        try:
            angle = request.angle
            speed = request.speed if request.speed > 0 else 50

            # Non-blocking call
            self.rob.moveTiltTo(angle, speed, False)

            response.success = True

        except Exception as e:
            self.get_logger().error(f"SetTilt failed: {e}")
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
                False  # non-blocking
            )
            response.success = True

        except Exception as e:
            self.get_logger().error(f'MoveWheelsTime failed: {e}')
            response.success = False

        return response
    
    # =========================
    # Wheels Actions
    # =========================
    def _move_wheels_deg_blocking(self, wheel, degrees, speed):
        try:
            self.rob.moveWheelsByDegrees(
                wheel,
                int(degrees),
                int(speed)
            )
        except Exception as e:
            self.get_logger().error(f'Blocking movement failed: {e}')
    
    def _move_wheels_time_blocking(self, right_speed, left_speed, duration):
        try:
            self.rob.moveWheelsByTime(
                right_speed,
                left_speed,
                duration,
                wait=True
            )
        except Exception as e:
            self.get_logger().error(f'Time movement failed: {e}')

    async def execute_move_wheels_degrees(self, goal_handle):
        request = goal_handle.request

        wheel = None
        try:
            wheel = Wheels[request.wheel]
        except KeyError:
            pass
        if wheel is None:
            goal_handle.abort()
            return MoveWheelsDegrees.Result(success=False)

        speed = request.speed
        degrees = request.degrees

        try:
            # ----------------------
            # Get initial position
            # ----------------------
            start_pos = self.rob.readWheelPosition(wheel)

            if not isinstance(start_pos, (int, float)):
                goal_handle.abort()
                return MoveWheelsDegrees.Result(success=False)

            target = start_pos + degrees

            # ----------------------
            # Start blocking call in thread
            # ----------------------
            movement_thread = threading.Thread(
                target=self._move_wheels_deg_blocking,
                args=(wheel, degrees, speed),
                daemon=True
            )
            movement_thread.start()

            feedback_msg = MoveWheelsDegrees.Feedback()

            # ----------------------
            # Monitor loop
            # ----------------------
            while movement_thread.is_alive():
                if goal_handle.is_cancel_requested:
                    self.rob.stopWheels()
                    goal_handle.canceled()
                    return MoveWheelsDegrees.Result(success=False)

                current = self.rob.readWheelPosition(wheel)

                if isinstance(current, (int, float)):
                    progress = abs(current - start_pos) / max(abs(degrees), 1e-5)

                    feedback_msg.progress = float(progress)
                    feedback_msg.current_position = float(current)

                    goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.05)

            # ----------------------
            # Movement finished
            # ----------------------
            goal_handle.succeed()
            return MoveWheelsDegrees.Result(success=True)

        except Exception as e:
            self.get_logger().error(f'Action failed: {e}')
            goal_handle.abort()
            return MoveWheelsDegrees.Result(success=False)

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


def main(args=None):
    rclpy.init(args=args)
    node = RoboboBaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
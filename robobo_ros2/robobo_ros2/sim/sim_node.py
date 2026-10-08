import math
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from robobo_ros2_interfaces.msg import RobotLocation
from robobo_ros2_interfaces.srv import (
    ChangeRobotLocation,
    SetRobotLocation,
    ResetSimulation,
)

from robobosim.RoboboSim import RoboboSim


class SimNode(Node):
    """
    ROS 2 node interfacing with RoboboSim (Unity simulator).

    Provides:
      - Topic:   /robobo/robot_<name>/sim/robot_location (robobo_ros2_interfaces/msg/RobotLocation)
      - Topic:   /robobo/robot_<name>/sim/location (alias)
      - Topic:   /robobo/robot_<name>/sim/pose (geometry_msgs/msg/Pose)
      - Service: /robobo/robot_<name>/sim/change_robot_location (robobo_ros2_interfaces/srv/ChangeRobotLocation)
      - Service: /robobo/robot_<name>/sim/set_robot_location (robobo_ros2_interfaces/srv/SetRobotLocation)
      - Service: /robobo/robot_<name>/sim/reset_simulation (robobo_ros2_interfaces/srv/ResetSimulation)
    """

    def __init__(self, robot_name='0', robot_id=0, ip='127.0.0.1', sim=None):
        super().__init__('sim_node')

        # Declare parameters
        self.declare_parameter('robot_name', str(robot_name))
        self.declare_parameter('robot_id', int(robot_id))
        self.declare_parameter('ip', str(ip))
        self.declare_parameter('frequency', 10.0)

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.ip = str(self.get_parameter('ip').value)
        self.frequency = float(self.get_parameter('frequency').value)

        self._namespace = f'/robobo/robot_{self.robot_name}/sim'

        # Establish connection to RoboboSim
        self._owns_sim = False
        if sim is not None:
            self.sim = sim
        else:
            self.sim = RoboboSim(self.ip)
            self._owns_sim = True
            try:
                self.sim.connect()
                self.get_logger().info(f'Connected to RoboboSim at {self.ip}:50505')
            except Exception as e:
                self.get_logger().warn(f'Could not connect to RoboboSim at {self.ip}:50505: {e}')

        # -------------------------
        # Publishers
        # -------------------------
        self.robot_location_pub = self.create_publisher(
            RobotLocation,
            f'{self._namespace}/robot_location',
            10
        )

        self.location_pub = self.create_publisher(
            RobotLocation,
            f'{self._namespace}/location',
            10
        )

        self.pose_pub = self.create_publisher(
            Pose,
            f'{self._namespace}/pose',
            10
        )

        # -------------------------
        # Services
        # -------------------------
        self.create_service(
            ChangeRobotLocation,
            f'{self._namespace}/change_robot_location',
            self.change_robot_location_cb
        )

        self.create_service(
            SetRobotLocation,
            f'{self._namespace}/set_robot_location',
            self.set_robot_location_cb
        )

        self.create_service(
            ResetSimulation,
            f'{self._namespace}/reset_simulation',
            self.reset_simulation_cb
        )

        # -------------------------
        # Location Updates
        # -------------------------
        try:
            self.sim.onNewLocation(self._on_sim_location_cb)
        except Exception as e:
            self.get_logger().warn(f'Failed to register RoboboSim onNewLocation callback: {e}')

        if self.frequency > 0.0:
            timer_period = 1.0 / self.frequency
            self.timer = self.create_timer(timer_period, self._poll_location)
        else:
            self.timer = None

        self.get_logger().info(f'SimNode started for robot_{self.robot_name} (id={self.robot_id}) on {self._namespace}')

    def _on_sim_location_cb(self):
        """Callback triggered by RoboboSim WebSocket upon receiving SIM-LOCATION."""
        self.read_and_publish_location()

    def _poll_location(self):
        """Periodic timer polling fallback to guarantee fresh location publication."""
        self.read_and_publish_location()

    def read_and_publish_location(self):
        """Query latest location from RoboboSim state and publish to topics."""
        try:
            loc = self.sim.getRobotLocation(self.robot_id)
            if loc is None:
                return

            pos = loc.get('position', {})
            rot = loc.get('rotation', {})

            px = float(pos.get('x', 0.0))
            py = float(pos.get('y', 0.0))
            pz = float(pos.get('z', 0.0))

            rx = float(rot.get('x', 0.0))
            ry = float(rot.get('y', 0.0))
            rz = float(rot.get('z', 0.0))

            # 1. Publish RobotLocation message
            msg = RobotLocation()
            msg.id = int(self.robot_id)
            msg.position.x = px
            msg.position.y = py
            msg.position.z = pz
            msg.rotation.x = rx
            msg.rotation.y = ry
            msg.rotation.z = rz

            self.robot_location_pub.publish(msg)
            self.location_pub.publish(msg)

            # 2. Publish standard geometry_msgs/Pose (convert Euler degrees -> quaternion)
            pose_msg = Pose()
            pose_msg.position.x = px
            pose_msg.position.y = py
            pose_msg.position.z = pz

            qx, qy, qz, qw = self.euler_to_quaternion(
                math.radians(rx),
                math.radians(ry),
                math.radians(rz)
            )
            pose_msg.orientation.x = qx
            pose_msg.orientation.y = qy
            pose_msg.orientation.z = qz
            pose_msg.orientation.w = qw

            self.pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().debug(f'Failed to publish robot location: {e}')

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles (in radians) to quaternion (x, y, z, w)."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def change_robot_location_cb(self, request, response):
        """Service callback to update robot location in RoboboSim."""
        try:
            pos = {
                'x': float(request.position.x),
                'y': float(request.position.y),
                'z': float(request.position.z)
            }
            rot = {
                'x': float(request.rotation.x),
                'y': float(request.rotation.y),
                'z': float(request.rotation.z)
            }
            self.sim.setRobotLocation(self.robot_id, position=pos, rotation=rot)
            response.success = True
            self.get_logger().info(
                f'Set robot_{self.robot_name} location to pos={pos}, rot={rot}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to change robot location: {e}')
            response.success = False
        return response

    def set_robot_location_cb(self, request, response):
        """Service callback alias for change_robot_location."""
        return self.change_robot_location_cb(request, response)

    def reset_simulation_cb(self, request, response):
        """Service callback to reset simulation scene in RoboboSim."""
        try:
            self.sim.resetSimulation()
            response.success = True
            self.get_logger().info('RoboboSim simulation reset successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to reset simulation: {e}')
            response.success = False
        return response

    def destroy_node(self):
        """Clean up resources on node shutdown."""
        if self._owns_sim and self.sim is not None:
            try:
                self.sim.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

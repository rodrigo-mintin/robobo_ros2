import math
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, Quaternion
from robobo_ros2_interfaces.msg import RobotLocation, SimObject, SimObjectArray
from robobo_ros2_interfaces.srv import (
    ChangeRobotLocation,
    SetRobotLocation,
    ResetSimulation,
    GetSimObjects,
    GetSimObjectLocation,
    SetSimObjectLocation,
)

from robobosim.RoboboSim import RoboboSim


class SimNode(Node):
    """
    ROS 2 node interfacing with RoboboSim (Unity simulator).

    Provides:
      - Robot Topics:
          /robobo/robot_<name>/sim/robot_location (robobo_ros2_interfaces/msg/RobotLocation)
          /robobo/robot_<name>/sim/location (alias)
          /robobo/robot_<name>/sim/pose (geometry_msgs/msg/Pose)
      - Object Topics:
          /robobo/robot_<name>/sim/objects (robobo_ros2_interfaces/msg/SimObjectArray)
          /robobo/robot_<name>/sim/object (robobo_ros2_interfaces/msg/SimObject)
          /robobo_sim/objects (global alias)
          /robobo_sim/object (global alias)
      - Robot Services:
          /robobo/robot_<name>/sim/change_robot_location (ChangeRobotLocation)
          /robobo/robot_<name>/sim/set_robot_location (SetRobotLocation)
          /robobo/robot_<name>/sim/reset_simulation (ResetSimulation)
      - Object Services:
          /robobo/robot_<name>/sim/get_objects (GetSimObjects)
          /robobo/robot_<name>/sim/get_object_location (GetSimObjectLocation)
          /robobo/robot_<name>/sim/set_object_location (SetSimObjectLocation)
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
        # Robot Publishers
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
        # Object Publishers
        # -------------------------
        self.objects_pub = self.create_publisher(
            SimObjectArray,
            f'{self._namespace}/objects',
            10
        )
        self.object_pub = self.create_publisher(
            SimObject,
            f'{self._namespace}/object',
            10
        )

        # Global aliases for convenience
        self.global_objects_pub = self.create_publisher(
            SimObjectArray,
            '/robobo_sim/objects',
            10
        )
        self.global_object_pub = self.create_publisher(
            SimObject,
            '/robobo_sim/object',
            10
        )

        # -------------------------
        # Robot Services
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
        # Object Services
        # -------------------------
        self.create_service(
            GetSimObjects,
            f'{self._namespace}/get_objects',
            self.get_objects_cb
        )
        self.create_service(
            GetSimObjects,
            '/robobo_sim/get_objects',
            self.get_objects_cb
        )

        self.create_service(
            GetSimObjectLocation,
            f'{self._namespace}/get_object_location',
            self.get_object_location_cb
        )
        self.create_service(
            GetSimObjectLocation,
            '/robobo_sim/get_object_location',
            self.get_object_location_cb
        )

        self.create_service(
            SetSimObjectLocation,
            f'{self._namespace}/set_object_location',
            self.set_object_location_cb
        )
        self.create_service(
            SetSimObjectLocation,
            '/robobo_sim/set_object_location',
            self.set_object_location_cb
        )

        # -------------------------
        # Location Updates
        # -------------------------
        try:
            self.sim.onNewLocation(self._on_sim_location_cb)
        except Exception as e:
            self.get_logger().warn(f'Failed to register RoboboSim onNewLocation callback: {e}')

        try:
            self.sim.onNewObjectLocation(self._on_sim_object_location_cb)
            if hasattr(self.sim, 'rem') and hasattr(self.sim.rem, 'processors'):
                obj_proc = self.sim.rem.processors.get('OBJ-LOCATION')
                if obj_proc and hasattr(obj_proc, 'callbacks'):
                    obj_proc.callbacks['object_location'] = self._on_sim_object_location_cb
                    obj_proc.callbacks['object-location'] = self._on_sim_object_location_cb
        except Exception as e:
            self.get_logger().warn(f'Failed to register onNewObjectLocation callback: {e}')

        if self.frequency > 0.0:
            timer_period = 1.0 / self.frequency
            self.timer = self.create_timer(timer_period, self._poll_location)
        else:
            self.timer = None

        self.get_logger().info(f'SimNode started for robot_{self.robot_name} (id={self.robot_id}) on {self._namespace}')

    def _on_sim_location_cb(self):
        """Callback triggered by RoboboSim WebSocket upon receiving SIM-LOCATION."""
        self.read_and_publish_location()

    def _on_sim_object_location_cb(self):
        """Callback triggered by RoboboSim WebSocket upon receiving SIM-OBJECT-LOCATION."""
        self.read_and_publish_objects()

    def _poll_location(self):
        """Periodic timer polling fallback to guarantee fresh location and object publication."""
        self.read_and_publish_location()
        self.read_and_publish_objects()

    def read_and_publish_location(self):
        """Query latest robot location from RoboboSim state and publish to topics."""
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

            qx, qy, qz, qw = self.euler_degrees_to_quaternion(rx, ry, rz)
            pose_msg.orientation.x = qx
            pose_msg.orientation.y = qy
            pose_msg.orientation.z = qz
            pose_msg.orientation.w = qw

            self.pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().debug(f'Failed to publish robot location: {e}')

    def _build_sim_object_msg(self, object_id: str, data: dict) -> SimObject:
        """Create a SimObject message from dictionary data."""
        obj_msg = SimObject()
        obj_msg.object_id = str(object_id)

        pos = data.get('position', {})
        rot = data.get('rotation', {})

        px = float(pos.get('x', 0.0))
        py = float(pos.get('y', 0.0))
        pz = float(pos.get('z', 0.0))

        rx = float(rot.get('x', 0.0))
        ry = float(rot.get('y', 0.0))
        rz = float(rot.get('z', 0.0))

        # Raw position
        obj_msg.position = Point(x=px, y=py, z=pz)

        # Raw Euler rotation in degrees
        obj_msg.rotation = Point(x=rx, y=ry, z=rz)

        # Standard ROS Pose
        qx, qy, qz, qw = self.euler_degrees_to_quaternion(rx, ry, rz)
        obj_msg.pose = Pose()
        obj_msg.pose.position.x = px
        obj_msg.pose.position.y = py
        obj_msg.pose.position.z = pz
        obj_msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        return obj_msg

    def read_and_publish_objects(self):
        """Query latest objects from RoboboSim state and publish."""
        try:
            if not hasattr(self.sim, 'rem') or not hasattr(self.sim.rem, 'state'):
                return

            state_objs = getattr(self.sim.rem.state, 'object_locations', {})
            if not state_objs:
                return

            arr = SimObjectArray()
            for obj_id, data in state_objs.items():
                if 'position' in data and 'rotation' in data:
                    obj_msg = self._build_sim_object_msg(obj_id, data)
                    arr.objects.append(obj_msg)

            self.objects_pub.publish(arr)
            self.global_objects_pub.publish(arr)

        except Exception as e:
            self.get_logger().debug(f'Failed to publish object locations: {e}')

    @staticmethod
    def euler_degrees_to_quaternion(rx: float, ry: float, rz: float):
        """Convert Euler angles in degrees (roll=rx, pitch=ry, yaw=rz) to quaternion (x, y, z, w)."""
        roll = math.radians(rx)
        pitch = math.radians(ry)
        yaw = math.radians(rz)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    @staticmethod
    def quaternion_to_euler_degrees(qx: float, qy: float, qz: float, qw: float):
        """Convert quaternion to Euler angles in degrees."""
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    # -------------------------
    # Robot Service Callbacks
    # -------------------------
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

    # -------------------------
    # Object Service Callbacks
    # -------------------------
    def get_objects_cb(self, request, response):
        """Returns list of currently known object IDs."""
        try:
            objs = self.sim.getObjects()
            response.object_ids = list(objs) if objs else []
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Failed to get objects: {e}')
            response.object_ids = []
            response.success = False
        return response

    def get_object_location_cb(self, request, response):
        """Returns raw position, rotation, and pose of the requested object."""
        try:
            loc = self.sim.getObjectLocation(request.object_id)
            if loc and 'position' in loc and 'rotation' in loc:
                obj_msg = self._build_sim_object_msg(request.object_id, loc)
                response.success = True
                response.message = f"Object '{request.object_id}' found."
                response.position = obj_msg.position
                response.rotation = obj_msg.rotation
                response.pose = obj_msg.pose
            else:
                response.success = False
                response.message = f"Object '{request.object_id}' not found."
                response.position = Point()
                response.rotation = Point()
                response.pose = Pose()
        except Exception as e:
            self.get_logger().error(f'Failed to get object location: {e}')
            response.success = False
            response.message = str(e)
            response.position = Point()
            response.rotation = Point()
            response.pose = Pose()
        return response

    def set_object_location_cb(self, request, response):
        """Sets object location in RoboboSim."""
        try:
            obj_id = request.object_id
            pos_dict = None
            rot_dict = None

            if request.use_pose:
                pos_dict = {
                    'x': float(request.pose.position.x),
                    'y': float(request.pose.position.y),
                    'z': float(request.pose.position.z)
                }
                rx, ry, rz = self.quaternion_to_euler_degrees(
                    request.pose.orientation.x,
                    request.pose.orientation.y,
                    request.pose.orientation.z,
                    request.pose.orientation.w
                )
                rot_dict = {'x': rx, 'y': ry, 'z': rz}
            else:
                apply_pos = request.set_position or (
                    not request.set_rotation and (
                        request.position.x != 0.0 or
                        request.position.y != 0.0 or
                        request.position.z != 0.0
                    )
                )
                apply_rot = request.set_rotation or (
                    not request.set_position and (
                        request.rotation.x != 0.0 or
                        request.rotation.y != 0.0 or
                        request.rotation.z != 0.0
                    )
                )

                if apply_pos:
                    pos_dict = {
                        'x': float(request.position.x),
                        'y': float(request.position.y),
                        'z': float(request.position.z)
                    }
                if apply_rot:
                    rot_dict = {
                        'x': float(request.rotation.x),
                        'y': float(request.rotation.y),
                        'z': float(request.rotation.z)
                    }

            self.sim.setObjectLocation(obj_id, position=pos_dict, rotation=rot_dict)
            response.success = True
            response.message = f"Set object '{obj_id}' location successfully."
        except Exception as e:
            self.get_logger().error(f'Failed to set object location: {e}')
            response.success = False
            response.message = str(e)
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

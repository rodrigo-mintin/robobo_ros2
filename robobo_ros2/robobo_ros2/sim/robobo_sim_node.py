#!/usr/bin/env python3
import math
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from std_srvs.srv import Trigger
import tf2_ros

try:
    from robobosim.RoboboSim import RoboboSim
except ImportError:
    RoboboSim = None

from robobo_ros2_interfaces.msg import SimObject, SimObjectArray
from robobo_ros2_interfaces.srv import (
    GetSimObjects,
    GetSimObjectLocation,
    SetSimObjectLocation
)


def euler_degrees_to_quaternion(rx: float, ry: float, rz: float):
    """Convert Euler angles in degrees (roll=rx, pitch=ry, yaw=rz) to quaternion."""
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


def quaternion_to_euler_degrees(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion to Euler angles in degrees (rx, ry, rz)."""
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


class RoboboSimNode(Node):
    """
    ROS 2 node interfacing RoboboSim simulation environment.
    Exposes object locations/rotations/poses via topics and services.
    """

    def __init__(self, sim_instance=None):
        super().__init__('robobo_sim_node')

        # --- Parameters ---
        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('sim_namespace', '/robobo_sim')
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('world_frame_id', 'world')

        self.ip = str(self.get_parameter('ip').value)
        self.sim_namespace = str(self.get_parameter('sim_namespace').value).rstrip('/')
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.world_frame_id = str(self.get_parameter('world_frame_id').value)

        self.lock = threading.Lock()
        self.objects_data = {}  # {object_id: {"position": {...}, "rotation": {...}}}

        # --- RoboboSim connection ---
        if sim_instance is not None:
            self.sim = sim_instance
        else:
            if RoboboSim is None:
                self.get_logger().error("robobosim library is not installed. Please install robobosim.")
                self.sim = None
            else:
                self.get_logger().info(f"Connecting to RoboboSim at {self.ip}...")
                self.sim = RoboboSim(self.ip)
                try:
                    self.sim.connect()
                    self.get_logger().info("Connected to RoboboSim successfully.")
                except Exception as e:
                    self.get_logger().error(f"Failed to connect to RoboboSim: {e}")

        # Register callback with RoboboSim (including workaround for library key mismatch bug)
        if self.sim is not None:
            self._register_callbacks()

        # --- Publishers ---
        self.objects_pub = self.create_publisher(
            SimObjectArray,
            f'{self.sim_namespace}/objects',
            10
        )
        self.object_pub = self.create_publisher(
            SimObject,
            f'{self.sim_namespace}/object',
            10
        )

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Services ---
        self.get_objects_srv = self.create_service(
            GetSimObjects,
            f'{self.sim_namespace}/get_objects',
            self._handle_get_objects
        )
        self.get_object_loc_srv = self.create_service(
            GetSimObjectLocation,
            f'{self.sim_namespace}/get_object_location',
            self._handle_get_object_location
        )
        self.set_object_loc_srv = self.create_service(
            SetSimObjectLocation,
            f'{self.sim_namespace}/set_object_location',
            self._handle_set_object_location
        )
        self.reset_sim_srv = self.create_service(
            Trigger,
            f'{self.sim_namespace}/reset_simulation',
            self._handle_reset_simulation
        )

        # Periodic publish timer
        if self.publish_frequency > 0:
            timer_period = 1.0 / self.publish_frequency
            self.timer = self.create_timer(timer_period, self._publish_objects_loop)

        self.get_logger().info(
            f"RoboboSimNode initialized on namespace: {self.sim_namespace} "
            f"(publish rate: {self.publish_frequency} Hz, tf: {self.publish_tf})"
        )

    def _register_callbacks(self):
        """Register simulation callbacks and apply compatibility patches."""
        try:
            self.sim.onNewObjectLocation(self._on_sim_object_location)
            # Patch known typo/key mismatch in robobosim remotelib vs ObjectLocationProcessor
            if hasattr(self.sim, 'rem') and hasattr(self.sim.rem, 'processors'):
                obj_proc = self.sim.rem.processors.get('OBJ-LOCATION')
                if obj_proc and hasattr(obj_proc, 'callbacks'):
                    obj_proc.callbacks['object_location'] = self._on_sim_object_location
                    obj_proc.callbacks['object-location'] = self._on_sim_object_location
        except Exception as e:
            self.get_logger().warn(f"Could not hook object location callback: {e}")

    def _on_sim_object_location(self):
        """Called when a new object location packet is received from RoboboSim."""
        self._sync_objects_from_sim()

    def _sync_objects_from_sim(self):
        """Read latest object locations from RoboboSim state and broadcast."""
        if not self.sim or not hasattr(self.sim, 'rem'):
            return

        with self.lock:
            try:
                state_objs = getattr(self.sim.rem.state, 'object_locations', {})
                for object_id, data in state_objs.items():
                    if 'position' in data and 'rotation' in data:
                        self.objects_data[str(object_id)] = {
                            'position': dict(data['position']),
                            'rotation': dict(data['rotation'])
                        }
            except Exception as e:
                self.get_logger().error(f"Error reading objects from sim: {e}")

    def _build_sim_object_msg(self, object_id: str, data: dict) -> SimObject:
        """Create a SimObject ROS 2 message from cached dictionary."""
        msg = SimObject()
        msg.object_id = str(object_id)

        pos = data.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0})
        rot = data.get('rotation', {'x': 0.0, 'y': 0.0, 'z': 0.0})

        # Raw position
        msg.position = Point()
        msg.position.x = float(pos.get('x', 0.0))
        msg.position.y = float(pos.get('y', 0.0))
        msg.position.z = float(pos.get('z', 0.0))

        # Raw Euler rotation (degrees)
        msg.rotation = Point()
        msg.rotation.x = float(rot.get('x', 0.0))
        msg.rotation.y = float(rot.get('y', 0.0))
        msg.rotation.z = float(rot.get('z', 0.0))

        # Standard ROS Pose (Point + Quaternion)
        qx, qy, qz, qw = euler_degrees_to_quaternion(
            msg.rotation.x, msg.rotation.y, msg.rotation.z
        )

        msg.pose = Pose()
        msg.pose.position.x = msg.position.x
        msg.pose.position.y = msg.position.y
        msg.pose.position.z = msg.position.z
        msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        return msg

    def _publish_objects_loop(self):
        """Periodically publish SimObjectArray and broadcast TFs."""
        self._sync_objects_from_sim()

        array_msg = SimObjectArray()
        stamp = self.get_clock().now().to_msg()

        with self.lock:
            for obj_id, data in self.objects_data.items():
                obj_msg = self._build_sim_object_msg(obj_id, data)
                array_msg.objects.append(obj_msg)

                if self.publish_tf:
                    t = TransformStamped()
                    t.header.stamp = stamp
                    t.header.frame_id = self.world_frame_id
                    t.child_frame_id = f"sim_object_{obj_id}"
                    t.transform.translation.x = obj_msg.position.x
                    t.transform.translation.y = obj_msg.position.y
                    t.transform.translation.z = obj_msg.position.z
                    t.transform.rotation = obj_msg.pose.orientation
                    self.tf_broadcaster.sendTransform(t)

        self.objects_pub.publish(array_msg)

    # --- Service Callbacks ---

    def _handle_get_objects(self, request, response):
        """Returns list of currently known object IDs."""
        self._sync_objects_from_sim()
        with self.lock:
            response.object_ids = list(self.objects_data.keys())
        response.success = True
        return response

    def _handle_get_object_location(self, request, response):
        """Returns raw position, rotation, and pose of the requested object."""
        self._sync_objects_from_sim()
        obj_id = request.object_id

        with self.lock:
            if obj_id in self.objects_data:
                obj_msg = self._build_sim_object_msg(obj_id, self.objects_data[obj_id])
                response.success = True
                response.message = f"Object '{obj_id}' found."
                response.position = obj_msg.position
                response.rotation = obj_msg.rotation
                response.pose = obj_msg.pose
            else:
                response.success = False
                response.message = f"Object '{obj_id}' not found in simulation."
                response.position = Point()
                response.rotation = Point()
                response.pose = Pose()

        return response

    def _handle_set_object_location(self, request, response):
        """Sets object location in RoboboSim."""
        if not self.sim:
            response.success = False
            response.message = "RoboboSim connection is not available."
            return response

        obj_id = request.object_id
        pos_dict = None
        rot_dict = None

        try:
            if request.use_pose:
                # Use geometry_msgs/Pose
                pos_dict = {
                    'x': float(request.pose.position.x),
                    'y': float(request.pose.position.y),
                    'z': float(request.pose.position.z)
                }
                rx, ry, rz = quaternion_to_euler_degrees(
                    request.pose.orientation.x,
                    request.pose.orientation.y,
                    request.pose.orientation.z,
                    request.pose.orientation.w
                )
                rot_dict = {'x': rx, 'y': ry, 'z': rz}
            else:
                # Use raw position / rotation
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

            with self.lock:
                self.sim.setObjectLocation(obj_id, position=pos_dict, rotation=rot_dict)

            response.success = True
            response.message = f"Successfully set location for object '{obj_id}'."
        except Exception as e:
            self.get_logger().error(f"Failed to set object location: {e}")
            response.success = False
            response.message = str(e)

        return response

    def _handle_reset_simulation(self, request, response):
        """Resets the simulation."""
        if not self.sim:
            response.success = False
            response.message = "RoboboSim connection is not available."
            return response

        try:
            with self.lock:
                self.sim.resetSimulation()
            response.success = True
            response.message = "Simulation reset requested."
        except Exception as e:
            self.get_logger().error(f"Failed to reset simulation: {e}")
            response.success = False
            response.message = str(e)

        return response

    def destroy_node(self):
        """Cleanup and disconnect RoboboSim."""
        if self.sim:
            try:
                self.sim.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboboSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

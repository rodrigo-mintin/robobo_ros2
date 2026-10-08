import math
from unittest.mock import MagicMock
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion

from robobo_ros2.sim.robobo_sim_node import (
    RoboboSimNode,
    euler_degrees_to_quaternion,
    quaternion_to_euler_degrees,
)
from robobo_ros2_interfaces.srv import (
    GetSimObjects,
    GetSimObjectLocation,
    SetSimObjectLocation,
)


def setup_module():
    if not rclpy.ok():
        rclpy.init()


def teardown_module():
    if rclpy.ok():
        rclpy.shutdown()


def test_euler_quaternion_conversion():
    # 0 rotation
    qx, qy, qz, qw = euler_degrees_to_quaternion(0.0, 0.0, 0.0)
    assert math.isclose(qw, 1.0, abs_tol=1e-5)
    assert math.isclose(qx, 0.0, abs_tol=1e-5)
    assert math.isclose(qy, 0.0, abs_tol=1e-5)
    assert math.isclose(qz, 0.0, abs_tol=1e-5)

    rx, ry, rz = quaternion_to_euler_degrees(qx, qy, qz, qw)
    assert math.isclose(rx, 0.0, abs_tol=1e-5)
    assert math.isclose(ry, 0.0, abs_tol=1e-5)
    assert math.isclose(rz, 0.0, abs_tol=1e-5)

    # 90 degrees around Z axis (yaw)
    qx, qy, qz, qw = euler_degrees_to_quaternion(0.0, 0.0, 90.0)
    rx, ry, rz = quaternion_to_euler_degrees(qx, qy, qz, qw)
    assert math.isclose(rx, 0.0, abs_tol=1e-5)
    assert math.isclose(ry, 0.0, abs_tol=1e-5)
    assert math.isclose(rz, 90.0, abs_tol=1e-5)


def test_sim_node_topics_and_services_setup():
    mock_sim = MagicMock()
    mock_sim.rem = MagicMock()
    mock_sim.rem.state = MagicMock()
    mock_sim.rem.state.object_locations = {
        "box_1": {
            "position": {"x": 1.5, "y": 0.2, "z": -3.0},
            "rotation": {"x": 0.0, "y": 45.0, "z": 0.0},
        },
        "cylinder_2": {
            "position": {"x": -2.0, "y": 0.0, "z": 1.0},
            "rotation": {"x": 10.0, "y": 0.0, "z": 90.0},
        },
    }

    node = RoboboSimNode(sim_instance=mock_sim)

    # Verify publishers and services
    assert node.objects_pub.topic_name == '/robobo_sim/objects'
    assert node.object_pub.topic_name == '/robobo_sim/object'
    assert node.get_objects_srv.service_name == '/robobo_sim/get_objects'
    assert node.get_object_loc_srv.service_name == '/robobo_sim/get_object_location'
    assert node.set_object_loc_srv.service_name == '/robobo_sim/set_object_location'

    # Test get_objects service callback
    get_objs_req = GetSimObjects.Request()
    get_objs_res = GetSimObjects.Response()
    node._handle_get_objects(get_objs_req, get_objs_res)
    assert get_objs_res.success is True
    assert set(get_objs_res.object_ids) == {"box_1", "cylinder_2"}

    # Test get_object_location service callback with separated raw values and pose
    get_loc_req = GetSimObjectLocation.Request()
    get_loc_req.object_id = "box_1"
    get_loc_res = GetSimObjectLocation.Response()
    node._handle_get_object_location(get_loc_req, get_loc_res)

    assert get_loc_res.success is True
    # Raw separated position
    assert math.isclose(get_loc_res.position.x, 1.5)
    assert math.isclose(get_loc_res.position.y, 0.2)
    assert math.isclose(get_loc_res.position.z, -3.0)
    # Raw separated rotation
    assert math.isclose(get_loc_res.rotation.x, 0.0)
    assert math.isclose(get_loc_res.rotation.y, 45.0)
    assert math.isclose(get_loc_res.rotation.z, 0.0)
    # Pose
    assert math.isclose(get_loc_res.pose.position.x, 1.5)
    assert get_loc_res.pose.orientation.w != 0.0

    # Test set_object_location callback with raw separated coordinates
    set_loc_req = SetSimObjectLocation.Request()
    set_loc_req.object_id = "box_1"
    set_loc_req.position = Point(x=2.0, y=0.5, z=4.0)
    set_loc_req.rotation = Point(x=0.0, y=180.0, z=0.0)
    set_loc_req.set_position = True
    set_loc_req.set_rotation = True
    set_loc_req.use_pose = False
    set_loc_res = SetSimObjectLocation.Response()

    node._handle_set_object_location(set_loc_req, set_loc_res)
    assert set_loc_res.success is True
    mock_sim.setObjectLocation.assert_called_with(
        "box_1",
        position={"x": 2.0, "y": 0.5, "z": 4.0},
        rotation={"x": 0.0, "y": 180.0, "z": 0.0},
    )

    # Test set_object_location callback with Pose
    mock_sim.setObjectLocation.reset_mock()
    set_pose_req = SetSimObjectLocation.Request()
    set_pose_req.object_id = "box_1"
    set_pose_req.use_pose = True
    set_pose_req.pose = Pose()
    set_pose_req.pose.position = Point(x=5.0, y=1.0, z=-2.0)
    qx, qy, qz, qw = euler_degrees_to_quaternion(0.0, 90.0, 0.0)
    set_pose_req.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    set_pose_res = SetSimObjectLocation.Response()

    node._handle_set_object_location(set_pose_req, set_pose_res)
    assert set_pose_res.success is True
    mock_sim.setObjectLocation.assert_called_once()
    call_args = mock_sim.setObjectLocation.call_args
    assert call_args[0][0] == "box_1"
    assert math.isclose(call_args[1]["position"]["x"], 5.0)
    assert math.isclose(call_args[1]["rotation"]["y"], 90.0, abs_tol=1e-3)

    node.destroy_node()

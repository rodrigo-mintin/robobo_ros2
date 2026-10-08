## Robobo ROS2

Some virtual nodes written in python (rclpy) and ROS2 Jazzy to communicate with the Robobo Ecosystem (Real robot and RoboboSim).

### What's in?

- **ROB Base**: Sensors (IR distance/Range, battery, pan/tilt positions, wheel encoders/speed), actuators (LEDs, pan/tilt movement, wheel movement services/actions), standard ROS2 `cmd_vel` velocity control, `odom` odometry publisher, and TF transform broadcasting.
- **Smartphone Modules**: Battery, IMU (orientation/acceleration), Brightness, Audio (sounds/notes), Speech (TTS), Noise and detected notes, Emotion display, Camera streaming (`Image` & `CameraInfo`) with camera controls, ArUco detection, QR detection, Color Blob detection, Object recognition, and Touch/Gesture detection (tap & fling).
- **Simulation Module (RoboboSim)**: Robot location topics (`robot_location`, `location`, `pose`), scene object tracking topics (`objects`, `object`), services to query and set object/robot locations (`get_objects`, `get_object_location`, `set_object_location`, `change_robot_location`, `set_robot_location`), and simulation reset (`reset_simulation`) via [robobosim.py](https://github.com/mintforpeople/robobosim.py).

### How do I do this?

Remember to have `robobopy`, `robobopy_videostream`, and `robobosim` installed in your python environment:
```bash
pip install -r robobo_ros2/requirements.txt
```
Or manually:
```bash
pip install robobopy
pip install robobopy_videostream
pip install robobosim
```

Build the workspace (builds both `robobo_ros2_interfaces` and `robobo_ros2`) and source the setup script:

**Linux / macOS (Bash):**
```bash
colcon build
source install/setup.bash
```

**Windows (PowerShell / CMD):**
```powershell
colcon build
# PowerShell
.\install\setup.ps1
# Or CMD
call install\setup.bat
```

Run the container node directly:
```bash
ros2 run robobo_ros2 robobo_container --ros-args -p ip:=IP_ROBOT -p robot_name:=ROBOT_NAME -p robot_id:=ROBOT_ID
```

#### Launch Parameters

Launch is done through command line arguments or via a YAML config file. A sample is provided in `robobo_ros2/config/sample.yaml`.

You can run the node with the YAML file as well:
```bash
ros2 run robobo_ros2 robobo_container --ros-args --params-file /path/to/params.yaml
```

Or using the launch file:
```bash
# Launch with defaults (IP: 127.0.0.1, robot_name: '0', robot_id: 0)
ros2 launch robobo_ros2 robobo.launch.py

# Launch with custom arguments
ros2 launch robobo_ros2 robobo.launch.py ip:=IP_ROBOT robot_name:=ROBOT_NAME robot_id:=ROBOT_ID

# Launch with a YAML parameter configuration file
ros2 launch robobo_ros2 robobo.launch.py params_file:=/path/to/params.yaml
```

If no modules are specified, all of them will be loaded, so the barebones parameters are the IP for the real robot and essentially no parameters for simulator.

```yaml
robobo_container:
  ros__parameters:
    robot_name: "0"
    ip: localhost
    robot_id: 0

    modules:
      - imu
      - brightness
      - speech
      - audio
      - noise
      - emotion
      - camera
      - qr
      - aruco
      - blob
      - object_recognition
      - touch
      - sim
```

| Parameter | Description |
| --- | --- |
| `robot_name` | Robot identifier within the ROS namespace (`/robobo/robot_<name>/...`). |
| `ip` | Robobo IP address (app IP or `localhost` / `127.0.0.1` for RoboboSim). |
| `robot_id` | Robot index for multi-robot simulation in RoboboSim (default: `0`). |
| `modules` | List of smartphone modules to load. |
| `cmd_vel_timeout` | *(Optional)* Safety watchdog timeout in seconds for `cmd_vel` velocity commands (default: `0.5`). |
| `frequency` | *(Optional)* Location polling frequency in Hz for the simulation node (default: `10.0`). |

### Simulation Module (RoboboSim)

When the `sim` module is active (enabled by default in `modules`), the container launches a `SimNode` that communicates with a running [RoboboSim](https://github.com/mintforpeople/robobosim.py) Unity instance over WebSocket (default port `50505`).

All simulation topics and services are namespaced under `/robobo/robot_<name>/sim/` and also exposed with global aliases under `/robobo_sim/` for scene-wide entities like objects.

#### Robot Topics

| Topic | Message Type | Description |
| --- | --- | --- |
| `/robobo/robot_<name>/sim/robot_location` | [`robobo_ros2_interfaces/msg/RobotLocation`](robobo_ros2_interfaces/msg/RobotLocation.msg) | Global coordinates (robot ID, position `(x, y, z)` and rotation `(pitch, yaw, roll)` in degrees). |
| `/robobo/robot_<name>/sim/location` | [`robobo_ros2_interfaces/msg/RobotLocation`](robobo_ros2_interfaces/msg/RobotLocation.msg) | Alias of `robot_location`. |
| `/robobo/robot_<name>/sim/pose` | [`geometry_msgs/msg/Pose`](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html) | Standard ROS 2 pose message with position and orientation quaternion (converted from Euler degrees). |

#### Object Topics

Simulation objects provide separated raw `position` and raw `rotation` (Euler degrees) directly from RoboboSim, alongside a standard ROS `Pose`. This makes distance and proximity calculations straightforward without needing to decompose orientation quaternions.

| Topic | Message Type | Description |
| --- | --- | --- |
| `/robobo/robot_<name>/sim/objects`<br>*(alias: `/robobo_sim/objects`)* | [`robobo_ros2_interfaces/msg/SimObjectArray`](robobo_ros2_interfaces/msg/SimObjectArray.msg) | Array of all active scene objects with raw `position`, raw `rotation`, and full `pose`. |
| `/robobo/robot_<name>/sim/object`<br>*(alias: `/robobo_sim/object`)* | [`robobo_ros2_interfaces/msg/SimObject`](robobo_ros2_interfaces/msg/SimObject.msg) | Individual object stream emitted whenever an object location update arrives. |

#### Robot Services

| Service | Service Type | Description |
| --- | --- | --- |
| `/robobo/robot_<name>/sim/change_robot_location` | [`robobo_ros2_interfaces/srv/ChangeRobotLocation`](robobo_ros2_interfaces/srv/ChangeRobotLocation.srv) | Repositions and rotates the robot in RoboboSim world coordinates. |
| `/robobo/robot_<name>/sim/set_robot_location` | [`robobo_ros2_interfaces/srv/SetRobotLocation`](robobo_ros2_interfaces/srv/SetRobotLocation.srv) | Alias for `change_robot_location`. |
| `/robobo/robot_<name>/sim/reset_simulation` | [`robobo_ros2_interfaces/srv/ResetSimulation`](robobo_ros2_interfaces/srv/ResetSimulation.srv) | Resets the simulation scene in RoboboSim to its initial state. |

#### Object Services

| Service | Service Type | Description |
| --- | --- | --- |
| `/robobo_sim/get_objects`<br>*(alias: `/robobo/robot_<name>/sim/get_objects`)* | [`robobo_ros2_interfaces/srv/GetSimObjects`](robobo_ros2_interfaces/srv/GetSimObjects.srv) | Returns list of all available object IDs in the scene. |
| `/robobo_sim/get_object_location`<br>*(alias: `/robobo/robot_<name>/sim/get_object_location`)* | [`robobo_ros2_interfaces/srv/GetSimObjectLocation`](robobo_ros2_interfaces/srv/GetSimObjectLocation.srv) | Queries a single object by `object_id` and returns separated `position`, `rotation`, and `pose`. |
| `/robobo_sim/set_object_location`<br>*(alias: `/robobo/robot_<name>/sim/set_object_location`)* | [`robobo_ros2_interfaces/srv/SetSimObjectLocation`](robobo_ros2_interfaces/srv/SetSimObjectLocation.srv) | Sets an object's location/rotation using separated raw coords or `Pose`. |

#### CLI Usage Examples

**Echo all objects in the scene:**
```bash
ros2 topic echo /robobo_sim/objects
```

**Get location of a single object by ID:**
```bash
ros2 service call /robobo_sim/get_object_location robobo_ros2_interfaces/srv/GetSimObjectLocation "{object_id: 'box_1'}"
```

**List all available scene objects:**
```bash
ros2 service call /robobo_sim/get_objects robobo_ros2_interfaces/srv/GetSimObjects "{}"
```

**Set object location (raw position & rotation):**
```bash
ros2 service call /robobo_sim/set_object_location robobo_ros2_interfaces/srv/SetSimObjectLocation "{object_id: 'box_1', position: {x: 2.0, y: 0.5, z: 1.0}, rotation: {x: 0.0, y: 90.0, z: 0.0}, set_position: true, set_rotation: true}"
```

**Echo robot location:**
```bash
ros2 topic echo /robobo/robot_0/sim/robot_location
```

**Echo standard robot Pose:**
```bash
ros2 topic echo /robobo/robot_0/sim/pose
```

**Change robot location and rotation:**
```bash
ros2 service call /robobo/robot_0/sim/change_robot_location robobo_ros2_interfaces/srv/ChangeRobotLocation "{position: {x: 1.0, y: 0.0, z: 2.0}, rotation: {x: 0.0, y: 90.0, z: 0.0}}"
```

**Reset simulation:**
```bash
ros2 service call /robobo/robot_0/sim/reset_simulation robobo_ros2_interfaces/srv/ResetSimulation "{}"
```

#### Measuring Object Distances in Python

Because `SimObject` provides separated `position` (`x, y, z`), distance calculations between objects (or between robot and object) can be performed directly without extracting positions from quaternions:

```python
import math
from robobo_ros2_interfaces.msg import SimObjectArray

def on_objects_received(msg: SimObjectArray):
    objs = {obj.object_id: obj for obj in msg.objects}
    if 'box_1' in objs and 'box_2' in objs:
        p1 = objs['box_1'].position
        p2 = objs['box_2'].position

        # 3D Euclidean distance
        dist_3d = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

        # 2D ground plane distance
        dist_2d = math.hypot(p1.x - p2.x, p1.z - p2.z)
```


### Running the demo

You can run the standalone demo script provided with the repo to test robot functionality (LEDs, wheel movements, pan/tilt motors).

The demo is a standalone client script that communicates with an already running `robobo_container` virtual node.

#### Step 1: Launch the Robobo container

In your first terminal, launch the container node (connecting to your real robot or simulator):

```bash
# Using launch file (defaults to IP 127.0.0.1, robot_name '0'):
ros2 launch robobo_ros2 robobo.launch.py

# Or with custom parameters:
ros2 launch robobo_ros2 robobo.launch.py ip:=ROBOBO_IP robot_name:=ROBOT_NAME robot_id:=ROBOT_ID

# Or directly with ros2 run:
ros2 run robobo_ros2 robobo_container --ros-args -p ip:=ROBOBO_IP -p robot_name:=ROBOT_NAME -p robot_id:=ROBOT_ID
```

#### Step 2: Run the demo script

While the virtual node is running, open a second terminal (sourced) and run the demo:

```bash
# Run with default robot_name '0':
ros2 run robobo_ros2 sample_demo

# Run with a specific robot name or timeout:
ros2 run robobo_ros2 sample_demo --robot-name ROBOT_NAME --timeout 15

# Or execute directly with python:
python src/robobo_ros2/robobo_ros2/sample_demo.py --robot-name ROBOT_NAME

# Show all available options:
ros2 run robobo_ros2 sample_demo --help
```
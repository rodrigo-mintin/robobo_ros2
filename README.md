## Robobo ROS2

Some virtual nodes written in python (rclpy) and ROS2 Jazzy to communicate with the Robobo Ecosystem (Real robot and RoboboSim).

### What's in?

- **ROB Base**: Sensors (IR distance/Range, battery, pan/tilt positions, wheel encoders/speed), actuators (LEDs, pan/tilt movement, wheel movement services/actions), standard ROS2 `cmd_vel` velocity control, `odom` odometry publisher, and TF transform broadcasting.
- **Smartphone Modules**: Battery, IMU (orientation/acceleration), Brightness, Audio (sounds/notes), Speech (TTS), Noise and detected notes, Emotion display, Camera streaming (`Image` & `CameraInfo`) with camera controls, ArUco detection, QR detection, Color Blob detection, Object recognition, and Touch/Gesture detection (tap & fling).

### How do I do this?

Remember to have `robobopy` and `robobopy_videostream` installed in your python environment:
```bash
pip install -r robobo_ros2/requirements.txt
```
Or manually:
```bash
pip install robobopy
pip install robobopy_videostream
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
```

| Parameter | Description |
| --- | --- |
| `robot_name` | Robot identifier within the ROS namespace (`/robobo/robot_<name>/...`). |
| `ip` | Robobo IP address (app IP or `localhost` / `127.0.0.1` for RoboboSim). |
| `robot_id` | Robot index for multi-robot simulation in RoboboSim (default: `0`). |
| `modules` | List of smartphone modules to load. |
| `cmd_vel_timeout` | *(Optional)* Safety watchdog timeout in seconds for `cmd_vel` velocity commands (default: `0.5`). |


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
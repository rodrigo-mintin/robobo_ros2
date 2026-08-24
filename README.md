## Robobo ROS2

Some virtual nodes written in python (rclpy) and ROS2 Jazzy to communicate with the Robobo Ecosystem (Real robot and RoboboSim).

### What's in?

- **ROB Base**: Sensors (IR distance/Range, battery, pan/tilt positions, wheel encoders/speed), actuators (LEDs, pan/tilt movement, wheel movement services/actions), standard ROS2 `cmd_vel` velocity control, `odom` odometry publisher, and TF transform broadcasting.
- **Smartphone Modules**: Battery, IMU (orientation/acceleration), Brightness, Audio (sounds/notes), Speech (TTS), Noise and detected notes, Emotion display, Camera streaming (`Image` & `CameraInfo`) with camera controls, ArUco detection, QR detection, Color Blob detection, Object recognition, and Touch/Gesture detection (tap & fling).

### How do I do this?

Remember to have `robobopy` and `robobopy_videostream` installed in your python environment:
```bash
pip install robobopy
pip install robobopy_videostream
```

ROS2 needs to include CVBridge. If that hasn't been installed yet, install it (e.g. `sudo apt install ros-jazzy-cv-bridge`).

Build the workspace and source the setup script:
```bash
colcon build
source install/setup.bash

ros2 run robobo_ros2 robobo_container --ros-args -p ip:=IP_ROBOT -p robot_name:=ROBOT_NAME -p robot_id:=ROBOT_ID
```

#### Launch Parameters

Launch is done through command line arguments or via a YAML config file. A sample is provided in `robobo_ros2/config/sample.yaml`:

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

### What's next?

- Nav2 navigation stack integration
- Expanded hardware testing and debugging


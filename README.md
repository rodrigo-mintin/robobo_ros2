## Robobo ROS2

Some virtual nodes written in python (rclpy) and ROS2 Jazzy to communicate with the Robobo Ecosystem (Real robot and RoboboSim)

### What's in?

Currently all the sensors and actuators from the ROB base are implemented and supported.
Some of the smartphone sensors have been implemented (IMU, Brightness, ARuco, QR, Blob, Speech, Audio and Emotion)

### How do I do this?

Remember to have robobopy installed in your python environment
```
pip install robobopy
pip install robobopy_videostream
```

ROS2 needs to include the CVBridge. If that hasn't been installed yet do it as so (eg. ``sudo apt install ros-jazzy-cv-bridge``)

Download the latest release and extract it in your ROS2 Workspace
Then just source your ROS2 environment and call the installation script
```
tar -xzf robobo_ros2_dist.tar.gz
source install/setup.bash

ros2 run robobo_ros2 robobo_base_node --ros-args -p ip:=IP_ROBOT -p robot_id:=ID_ROBOT
```

#### Launch Parameters

Launch is now done easily through a YAML config file. There is a sample within the project `robobo_ros2/config/sample.yaml`

```
robobo_container:
  ros__parameters:
    robot_name: "0"
    ip: localhost

    modules:
     - imu
     - brightness
     - speech
     - audio
     - emotion
     - qr
     - aruco
     - blob
```

### What's next?

- Finish implementing smartphone sensors and actuators
- Proper debug and testing


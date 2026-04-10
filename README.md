## Robobo ROS2

Some virtual nodes written in python (rclpy) and ROS2 Jazzy to communicate with the Robobo Ecosystem (Real robot and RoboboSim)

### What's in?

Currently all the sensors and actuators from the ROB base are implemented and supported

### How do I do this?

Remember to have robobopy installed in your python environment
``pip install robobopy``

Download the latest release and extract it in your ROS2 Workspace
Then just source your ROS2 environment and call the installation script
```
tar -xzf robobo_ros2_dist.tar.gz
source install/setup.bash

ros2 run robobo_ros2 robobo_base_node --ros-args -p ip:=IP_ROBOT -p robot_id:=ID_ROBOT
```

#### Launch Parameters

| Parameter    | Effect |
| -------- | ------- |
| ip  | Robobo's IP, use the app's provided IP or 'localhost' for RoboboSim   |
| robot_id | For multirobot purposes within RoboboSim. In multi-robot simulation, use 0 for the first robot, 1 for the second, etc. |

### What's next?

- Long blocking actions for ROB base methods (MoveWheelsByDegrees is planned as an action as it always blocks)
- Smartphone sensors (IMU is particularly interesting)
- There aren't smartphone actuators but some QoL services will be implemented to interact with camera sensors


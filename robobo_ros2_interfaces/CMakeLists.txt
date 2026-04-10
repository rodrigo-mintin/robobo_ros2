cmake_minimum_required(VERSION 3.8)
project(robobo_ros2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLed.srv"
  "srv/SetPan.srv"
  "srv/SetTilt.srv"

  "srv/MoveWheels.srv"
  "srv/MoveWheelsTime.srv"
  "srv/StopWheels.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
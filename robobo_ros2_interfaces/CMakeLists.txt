cmake_minimum_required(VERSION 3.8)
project(robobo_ros2_interfaces)

# -------------------------------
# Dependencies
# -------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# -------------------------------
# Collect interface files
# -------------------------------
file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
file(GLOB ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

# Ensure CMake re-runs when new files are added
set_property(DIRECTORY PROPERTY CMAKE_CONFIGURE_DEPENDS srv action)

# -------------------------------
# Generate interfaces
# -------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  ${ACTION_FILES}
  ${MSG_FILES}
  DEPENDENCIES action_msgs
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# -------------------------------
# Export runtime dependencies
# -------------------------------
ament_export_dependencies(rosidl_default_runtime)

# -------------------------------
# Package
# -------------------------------
ament_package()
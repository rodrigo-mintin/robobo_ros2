# Identifier of the simulation object
string object_id

# Raw translation/location coordinates (x, y, z) directly from RoboboSim
geometry_msgs/Point position

# Raw Euler rotation angles (x, y, z) directly from RoboboSim (in degrees)
geometry_msgs/Point rotation

# Standard ROS 2 Pose (Point position + Quaternion orientation)
geometry_msgs/Pose pose

# Custom message for robot state information
# Represents the current state of the robot for AI awareness

string robot_id
builtin_interfaces/Time timestamp
geometry_msgs/Pose pose
sensor_msgs/JointState joint_states
geometry_msgs/Twist velocity
string[] active_behaviors
float64 battery_level
bool is_connected
bool is_simulated
string status
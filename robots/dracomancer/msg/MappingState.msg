# Local derivative of the requested morphology, before feasibility/rate limiting.
# header.stamp is the device measurement time (not the publication time).
std_msgs/Header header
string[] robot_joints
string[] device_joints
# Row-major dq_robot / dq_device: len(robot_joints) x len(device_joints).
float64[] jacobian

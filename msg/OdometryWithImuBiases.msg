# Header
std_msgs/Header header
string child_frame_id

# Arbitrary Odometry Estimate
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

# IMU Biases
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gyro_bias

# Odometry state
uint8 odometry_state

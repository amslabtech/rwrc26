# Experimental conditional estimate; never used for control or goal switching.
# Covariance is conditional on assumed XY anchors and the initial heading alignment.
std_msgs/Header header
uint32 current_node_idx
uint32 subgoal_idx
# Initial, ordinary arrival or inconsistent recovery XY anchor; heading is not reanchored.
uint32 anchor_node_idx
time anchor_timestamp
# Last propagated pose time. May be older than header.stamp after discontinuity.
time pose_timestamp
uint32 anchor_epoch
uint32 odometry_epoch
bool diagnostic_only
bool heading_reference_assumed
bool odometry_continuous
float64 anchor_node_probability
string status
# Robot pose uses this synthetic map frame. No TF is broadcast.
string map_frame
geometry_msgs/PoseWithCovariance robot_pose
# Relative goal uses header.frame_id: x forward, y left, yaw CCW (rad).
geometry_msgs/PoseWithCovariance goal_pose
# Difference from the 1D MAP cell. Not an independent observation or correction.
float64 belief_position_disagreement
float64 belief_yaw_disagreement

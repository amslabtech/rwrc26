# Split navigation: localization (navigate.py --nav-role localization, parrot) -> waypoint
# inference (--nav-role waypoint, Xavier). One message per localized camera image.
# header.stamp is that image's stamp; header.frame_id is the odometry body frame.
std_msgs/Header header
uint32 current_node_idx
uint32 subgoal_idx
# Final goal reached: the waypoint node publishes a zero waypoint and stops.
bool reached_goal
# Relative subgoal pose at header.stamp (x forward, y left in m, yaw CCW in rad), from the
# local pose tracker. Only valid when goal_pose_valid; otherwise use the subgoal image only.
bool goal_pose_valid
float64 goal_x
float64 goal_y
float64 goal_yaw

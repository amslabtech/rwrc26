# Image heading alignment at subgoal arrivals (navigate.py --heading-align), every localization cycle.
# While active, the path adapter replaces the waypoint path with an in-place rotation of
# the robot pose to target_yaw (WAITING_IMAGE holds the current heading).
std_msgs/Header header        # image time; frame_id = odometry frame of the poses below
bool active
string status                 # IDLE, ALIGNED, ALIGNED_ODOM, ALIGNED_TAUGHT, ROTATING, ROTATING_ODOM, WAITING_IMAGE, TIMEOUT
uint32 current_node_idx
uint32 subgoal_idx
int32 node_idx                # node whose recorded heading is matched, -1 when idle
float64 dpsi                  # live - node heading from the image, rad (CCW +); NaN without a measurement
uint32 support                # inlier matches of that measurement
string source                 # image or odometry (taught-step fallback)
geometry_msgs/Pose2D robot    # odometry pose at header.stamp
float64 target_yaw            # odometry frame, rad

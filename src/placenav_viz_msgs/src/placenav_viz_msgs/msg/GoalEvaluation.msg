# Goal scores and selector decisions. Not a geometric pose observation.
# Position means 1D route progress; heading has no uncertainty distribution.
time query_timestamp
uint32 current_node_idx
uint32 subgoal_idx
string subgoal_update
uint32 odometry_epoch
float64 position_tolerance
float64 yaw_tolerance
float64 neighborhood_radius
bool heading_reference_assumed
float64 goal_s
float64 goal_yaw
float64 map_s
float64 map_yaw
float64 map_yaw_error
float64 position_score
float64 arrival_score
float64 neighborhood_mass
float64 local_passage_score
float64 local_passage_heading_score
float64 q10_s
float64 q50_s
float64 q90_s
# Kept for older tools; zero since overflow positions are retained, not clipped.
float64 boundary_clipped_mass
float64 endpoint_mass
# Probability outside the recorded route, including sub-cell Gaussian tails.
float64 outside_mass
float64 zero_node_fraction
# Scores above describe subgoal_idx (the image actually published).
# Decision fields below describe the goal evaluated BEFORE a possible switch.
string selection_status
int32 decision_goal_idx
float64 decision_score
uint32 confirmation_count
int32 recovery_candidate_idx
uint32 recovery_confirmation_count
float64 recovery_forward_score
float64 recovery_backward_score

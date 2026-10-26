# One candidate gap of a planning tick, as offered to the operator.

int32   traj_idx               # index into the tick's gap tubes / gap trajectories (sparse)
int32   display_id             # number drawn next to the candidate (dense); -1 if not shown
int32   left_model_id          # estimator id of the left gap point (stable over time)
int32   right_model_id         # estimator id of the right gap point
bool    feasible               # a non-empty trajectory exists
bool    shown                  # drawn as selectable this tick
uint16  tube_len               # number of gaps in the gap tube

# gap geometry, robot frame, at the gap snapshot stamp
float32 left_range
float32 left_theta
float32 right_range
float32 right_theta
float32 bearing                # bearing of the gap centre

# gap point estimator states, robot frame, at the gap snapshot stamp
geometry_msgs/Vector3 left_model_pos
geometry_msgs/Vector3 left_model_vel
geometry_msgs/Vector3 right_model_pos
geometry_msgs/Vector3 right_model_vel

# odom frame, at the tick
geometry_msgs/Point   goal_pos         # gap goal the trajectory aims at
geometry_msgs/Pose    terminal_pose    # last pose of the trajectory; zero if not feasible
geometry_msgs/Point[] path             # trajectory, downsampled
float32 cursor_dist                    # cursor to terminal pose (m); NaN if no cursor or not feasible
float32 cursor_bearing_diff            # bearing(cursor) - bearing(terminal pose), from the robot (rad); NaN if none

# DynamicGap's cost of this candidate's trajectory, as pickTraj computes it (lower is better)
float32 dg_cost                        # dg_terminal_cost + dg_clearance_cost; inf if the path has < 2 poses; NaN if not scored
float32 dg_terminal_cost               # Q_f * distance(terminal pose, local waypoint)
float32 dg_clearance_cost              # mean pose-wise cost against the propagated scans

# learned gap scorer (/learned_gap_scorer): its score of this candidate (higher is chosen);
# NaN if the scorer did not run this tick
float32 model_score

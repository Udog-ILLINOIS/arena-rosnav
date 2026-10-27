# One planning tick of the dynamic_gap planner in manual gap selection mode.
# Published on every tick, including ticks that end before candidates exist.
# A tick is a usable expert label when
#   source == SOURCE_GAP_CURSOR && cursor_live && !in_reset_window

uint16 SCHEMA_VERSION = 3   # 2: dg_pick_idx and per-candidate dg_* costs; 3: per-candidate model_score

uint8 SOURCE_NONE          = 0   # no selection this tick (learned mode: the scorer is not loaded)
uint8 SOURCE_GAP_CURSOR    = 1   # a gap was selected from the cursor
uint8 SOURCE_CLICK_TO_MOVE = 2   # following a click-created trajectory
uint8 SOURCE_MODEL         = 3   # the learned gap scorer chose (/learned_gap_scorer); selected_idx -1 and hold_still: it had no usable choice

uint8 STATUS_PLANNED       = 0   # candidates were built
uint8 STATUS_NOT_READY     = 1   # no scan, goal or initialisation yet
uint8 STATUS_COLLIDING     = 2   # robot in collision, planning skipped
uint8 STATUS_NO_GAPS       = 3   # no gaps detected

uint16  schema_version
Header  header                   # planning tick, sim time; frame_id = odom frame
time    wall_stamp
uint32  tick_seq                 # increments on every tick
int32   episode_id               # last /scenario_reset id; -1 before the first
bool    in_reset_window          # tick close to an episode reset (between scenario_reset_begin and shortly after scenario_reset)
uint8   status

# observation
sensor_msgs/LaserScan scan       # the scan the gaps were computed from
time    gap_snapshot_stamp       # stamp of that scan
time    odom_stamp
geometry_msgs/Pose    robot_pose_odom
geometry_msgs/Twist   robot_vel  # robot frame
geometry_msgs/Point   goal_robot_frame
geometry_msgs/Point   local_waypoint_robot_frame
ManualGapCandidate[]  candidates
bool    ungap_would_apply        # reserved (ungap generation is disabled); always false
bool    idle_would_apply         # reserved (idling generation is disabled); always false

# cursor
CursorState cursor               # latest cursor message received, verbatim
time    cursor_rx_wall           # planner wall clock when it was received; zero if never
geometry_msgs/Point cursor_odom  # point used for selection, odom frame
bool    cursor_valid             # a cursor point was available for selection
bool    cursor_live              # tool active, pick ok, and the last message is recent (wall clock)

# choice
uint8   source
int32   selected_idx             # position in candidates[]; -1 if none
int32   selected_left_model_id
int32   selected_right_model_id
geometry_msgs/Point click_point_odom   # target of SOURCE_CLICK_TO_MOVE
bool    hold_still               # the robot was commanded to stand still this tick

# DynamicGap's own choice (pickTraj on this tick's costs), recorded only; it is not executed
int32   dg_pick_idx              # position in candidates[] of the lowest-cost gap trajectory; -1 if none or not planned

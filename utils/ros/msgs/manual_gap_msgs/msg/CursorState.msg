# Manual gap selection cursor, published by the RViz ContinuousGapSelectionTool
# at 10 Hz (wall clock) while the tool is active, and once with
# tool_active = false when it is deactivated.

Header  header                 # sim time at publish; frame_id = RViz fixed frame
time    wall_stamp             # wall clock at publish
bool    tool_active            # false only in the message sent on deactivate
bool    pick_ok                # the mouse ray hits the ground plane (z = 0) inside the view
geometry_msgs/Point point      # ground point under the mouse, in header.frame_id; valid only if pick_ok
int32   screen_x               # mouse position in the render panel (pixels); -1 if unknown
int32   screen_y
time    last_move_wall         # wall clock of the last mouse move over the panel

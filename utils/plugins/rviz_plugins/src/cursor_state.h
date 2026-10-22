#ifndef RVIZ_PLUGINS_CURSOR_STATE_H
#define RVIZ_PLUGINS_CURSOR_STATE_H

#include <ros/time.h>
#include <manual_gap_msgs/CursorState.h>

#include <string>

namespace rviz
{
/**
 * Topic name under a configurable prefix: "/rto" or "/rto/" + "x" -> "/rto/x".
 * An empty prefix gives the bare (relative) name.
 */
inline std::string prefixedTopic(std::string prefix, const std::string& name)
{
  if (!prefix.empty() && prefix.back() != '/')
    prefix += '/';
  return prefix + name;
}

/**
 * Cursor message for one publish of the gap selection tool.
 * pick_ok: the mouse ray hit the ground plane inside the view; the point is
 * only filled in when it did.
 */
inline manual_gap_msgs::CursorState makeCursorState(bool tool_active, bool pick_ok,
                                                    double x, double y, double z,
                                                    int screen_x, int screen_y,
                                                    const std::string& frame_id,
                                                    const ros::Time& stamp,
                                                    const ros::WallTime& wall_now,
                                                    const ros::WallTime& last_move_wall)
{
  manual_gap_msgs::CursorState msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.wall_stamp = ros::Time(wall_now.sec, wall_now.nsec);
  msg.tool_active = tool_active;
  msg.pick_ok = tool_active && pick_ok;
  if (msg.pick_ok)
  {
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = z;
  }
  msg.screen_x = screen_x;
  msg.screen_y = screen_y;
  msg.last_move_wall = ros::Time(last_move_wall.sec, last_move_wall.nsec);
  return msg;
}

}  // namespace rviz

#endif

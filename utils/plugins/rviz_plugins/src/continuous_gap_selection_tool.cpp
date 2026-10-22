#include <OgrePlane.h>
#include <OgreVector3.h>
#include <QCursor>
#include <rviz/ogre_helpers/ogre_vector.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/properties/string_property.h>
#include <geometry_msgs/PointStamped.h>
#include <manual_gap_msgs/CursorState.h>
#include "continuous_gap_selection_tool.h"
#include "cursor_state.h"
#include <sstream>

namespace rviz
{

ContinuousGapSelectionTool::ContinuousGapSelectionTool()
  : isPublishing_(false), viewport_(nullptr), screenX_(-1), screenY_(-1), hasScreenPos_(false)
{
  shortcut_key_ = 'g';

  topic_prefix_property_ = new StringProperty("Topic Prefix", "/rto/",
                                              "Namespace of the robot whose planner receives the cursor "
                                              "(gap_selection_cursor) and click-to-move points (traj_creation_point).",
                                              getPropertyContainer(), SLOT(updateTopics()), this);
}

void ContinuousGapSelectionTool::onInitialize()
{
  hit_cursor_ = cursor_;
  std_cursor_ = getDefaultCursor();
  updateTopics();
  isPublishing_ = false;
  hasScreenPos_ = false;
  // wall clock: keeps publishing at 10 Hz whether the simulation runs, is
  // paused or runs faster than real time
  publishTimer_ = nh_.createWallTimer(ros::WallDuration(0.1),
                                      &ContinuousGapSelectionTool::timerCallback, this, false, false);
}

void ContinuousGapSelectionTool::updateTopics()
{
  std::string prefix = topic_prefix_property_->getStdString();
  cursor_pub_ = nh_.advertise<manual_gap_msgs::CursorState>(prefixedTopic(prefix, "gap_selection_cursor"), 1);
  trajpub_ = nh_.advertise<geometry_msgs::PointStamped>(prefixedTopic(prefix, "traj_creation_point"), 1);
}

bool ContinuousGapSelectionTool::pickGround(int x, int y, double& gx, double& gy)
{
  if (!viewport_)
    return false;

  Ogre::Plane ground(Ogre::Vector3::UNIT_Z, 0.0f);
  Ogre::Vector3 hit;
  if (!getPointOnPlaneFromWindowXY(viewport_, ground, x, y, hit))
    return false;

  gx = hit.x;
  gy = hit.y;
  return true;
}

bool ContinuousGapSelectionTool::pointerInPanel()
{
  // no mouse event arrives once the pointer has left the panel
  if (!panel_)
    return false;
  return panel_->rect().contains(panel_->mapFromGlobal(QCursor::pos()));
}

void ContinuousGapSelectionTool::publishCursor(bool tool_active)
{
  double gx = 0.0, gy = 0.0;
  bool pick_ok = tool_active && hasScreenPos_ && pointerInPanel() && pickGround(screenX_, screenY_, gx, gy);

  cursor_pub_.publish(makeCursorState(tool_active, pick_ok, gx, gy, 0.0,
                                      hasScreenPos_ ? screenX_ : -1, hasScreenPos_ ? screenY_ : -1,
                                      context_->getFixedFrame().toStdString(),
                                      ros::Time::now(), ros::WallTime::now(), lastMoveWall_));
}

void ContinuousGapSelectionTool::timerCallback(const ros::WallTimerEvent&)
{
  if (isPublishing_)
    publishCursor(true);
}

void ContinuousGapSelectionTool::activate()
{
  isPublishing_ = true;
  publishTimer_.start();
}

void ContinuousGapSelectionTool::deactivate()
{
  isPublishing_ = false;
  publishTimer_.stop();
  // tells the planner the cursor is gone
  publishCursor(false);
}

int ContinuousGapSelectionTool::processMouseEvent(ViewportMouseEvent& event)
{
  int flags = 0;

  panel_ = event.panel;
  viewport_ = event.viewport;
  screenX_ = event.x;
  screenY_ = event.y;
  hasScreenPos_ = true;
  if (event.type == QEvent::MouseMove)
    lastMoveWall_ = ros::WallTime::now();

  double gx = 0.0, gy = 0.0;
  bool success = pickGround(event.x, event.y, gx, gy);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (success)
  {
    std::ostringstream s;
    s << "<b>Hover:</b> select the gap nearest the cursor. <b>Left-Click:</b> drive to this point.";
    s.precision(3);
    s << " [" << gx << "," << gy << "]";
    setStatus(s.str().c_str());

    if (event.leftDown()) {
      geometry_msgs::PointStamped trajPt;
      trajPt.point.x = gx;
      trajPt.point.y = gy;
      trajPt.point.z = 0.0;
      trajPt.header.frame_id = context_->getFixedFrame().toStdString();
      trajPt.header.stamp = ros::Time::now();

      trajpub_.publish(trajPt);
      ROS_INFO_STREAM("ContinuousGapSelectionTool: published traj creation point ["
                       << gx << ", " << gy << "]");
    }
  }
  else
  {
    setStatus("Point at the ground to select a gap.");
  }

  return flags;
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::ContinuousGapSelectionTool, rviz::Tool)

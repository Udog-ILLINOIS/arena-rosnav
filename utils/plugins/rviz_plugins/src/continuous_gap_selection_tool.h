#ifndef CONTINUOUS_GAP_SELECTION_TOOL_H
#define CONTINUOUS_GAP_SELECTION_TOOL_H
#include <geometry_msgs/PointStamped.h>
#ifndef Q_MOC_RUN
# include <QObject>
# include <QPointer>
# include <ros/ros.h>
# include "rviz/tool.h"
#endif

namespace Ogre
{
class Viewport;
}

namespace rviz
{
class RenderPanel;
class StringProperty;

/**
 * Manual gap selection. While active, publishes the ground point under the
 * mouse as manual_gap_msgs/CursorState on <prefix>gap_selection_cursor at
 * 10 Hz (wall clock). The screen position is kept and re-projected onto the
 * ground plane at every publish, so a still mouse keeps pointing at what is
 * under it on screen while the camera follows the robot. Left-click publishes
 * a click-to-move point on <prefix>traj_creation_point.
 */
class ContinuousGapSelectionTool : public Tool
{
Q_OBJECT
public:
  ContinuousGapSelectionTool();
  virtual ~ContinuousGapSelectionTool() {}
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual int processMouseEvent(ViewportMouseEvent& event);

private Q_SLOTS:
  void updateTopics();

private:
  void timerCallback(const ros::WallTimerEvent&);
  void publishCursor(bool tool_active);
  bool pickGround(int x, int y, double& gx, double& gy);
  bool pointerInPanel();

  ros::NodeHandle nh_;
  ros::Publisher cursor_pub_;
  ros::Publisher trajpub_;
  StringProperty* topic_prefix_property_;
  bool isPublishing_;
  QCursor std_cursor_;
  QCursor hit_cursor_;
  ros::WallTimer publishTimer_;

  // last mouse position over a render panel; re-projected at every publish
  QPointer<RenderPanel> panel_;
  Ogre::Viewport* viewport_;
  int screenX_;
  int screenY_;
  bool hasScreenPos_;
  ros::WallTime lastMoveWall_;
};
} // end namespace rviz
#endif

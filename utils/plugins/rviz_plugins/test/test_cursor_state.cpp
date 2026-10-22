#include <gtest/gtest.h>
#include <cmath>

#include "cursor_state.h"

using rviz::makeCursorState;
using rviz::prefixedTopic;

TEST(PrefixedTopic, JoinsWithOneSlash)
{
  EXPECT_EQ("/rto/gap_selection_cursor", prefixedTopic("/rto/", "gap_selection_cursor"));
  EXPECT_EQ("/rto/gap_selection_cursor", prefixedTopic("/rto", "gap_selection_cursor"));
  EXPECT_EQ("/gap_selection_cursor", prefixedTopic("/", "gap_selection_cursor"));
  EXPECT_EQ("gap_selection_cursor", prefixedTopic("", "gap_selection_cursor"));
}

TEST(MakeCursorState, ActiveWithPick)
{
  ros::WallTime now(100, 5), moved(90, 7);
  auto msg = makeCursorState(true, true, 1.5, -2.0, 0.0, 10, 20, "map", ros::Time(3, 4), now, moved);
  EXPECT_TRUE(msg.tool_active);
  EXPECT_TRUE(msg.pick_ok);
  EXPECT_DOUBLE_EQ(1.5, msg.point.x);
  EXPECT_DOUBLE_EQ(-2.0, msg.point.y);
  EXPECT_EQ("map", msg.header.frame_id);
  EXPECT_EQ(ros::Time(3, 4), msg.header.stamp);
  EXPECT_EQ(ros::Time(100, 5), msg.wall_stamp);
  EXPECT_EQ(ros::Time(90, 7), msg.last_move_wall);
  EXPECT_EQ(10, msg.screen_x);
  EXPECT_EQ(20, msg.screen_y);
}

TEST(MakeCursorState, FailedPickHasNoPoint)
{
  auto msg = makeCursorState(true, false, 1.5, -2.0, 0.0, 10, 20, "map", ros::Time(3, 4),
                             ros::WallTime(100, 0), ros::WallTime(90, 0));
  EXPECT_TRUE(msg.tool_active);
  EXPECT_FALSE(msg.pick_ok);
  EXPECT_DOUBLE_EQ(0.0, msg.point.x);
  EXPECT_DOUBLE_EQ(0.0, msg.point.y);
}

TEST(MakeCursorState, InactiveIsNeverPickOk)
{
  auto msg = makeCursorState(false, true, 1.5, -2.0, 0.0, -1, -1, "map", ros::Time(3, 4),
                             ros::WallTime(100, 0), ros::WallTime(90, 0));
  EXPECT_FALSE(msg.tool_active);
  EXPECT_FALSE(msg.pick_ok);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

cmake_minimum_required(VERSION 3.0.2)
project(manual_gap_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(FILES
  CursorState.msg
  ManualGapCandidate.msg
  ManualGapSample.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

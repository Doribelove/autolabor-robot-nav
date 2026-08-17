cmake_minimum_required(VERSION 3.0.2)
project(autolabor_fod_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  FodDetection.msg
  FodDetectionArray.msg
  FodGroundObservation.msg
  FodGroundObservationArray.msg
  FodTarget.msg
  FodTrack.msg
  FodTrackArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  sensor_msgs
  std_msgs
)

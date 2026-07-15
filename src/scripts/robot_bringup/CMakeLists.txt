cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_self_filter src/pointcloud_self_filter.cpp)
target_link_libraries(pointcloud_self_filter ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/autolabor_cmd_test.py
  scripts/autolabor_keyboard_teleop.py
  scripts/gps_goal_speed_limiter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS pointcloud_self_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

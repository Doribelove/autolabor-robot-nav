cmake_minimum_required(VERSION 3.0.2)
project(gps_module)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_localization_node.py
  scripts/gps_goal_node.py
  scripts/gps_goal_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

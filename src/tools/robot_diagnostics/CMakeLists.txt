cmake_minimum_required(VERSION 3.0.2)
project(robot_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/check_can.py
  scripts/gps_static_error_monitor.py
  scripts/check_odom.py
  scripts/check_tf.py
  scripts/wait_for_topics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

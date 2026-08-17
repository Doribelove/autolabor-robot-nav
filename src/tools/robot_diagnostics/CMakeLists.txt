cmake_minimum_required(VERSION 3.0.2)
project(robot_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  autolabor_canbus_driver
  geometry_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    autolabor_canbus_driver
    geometry_msgs
    nav_msgs
    rosgraph
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_steer_center_calibration.py)
endif()

catkin_install_python(PROGRAMS
  scripts/check_can.py
  scripts/gps_static_error_monitor.py
  scripts/check_odom.py
  scripts/check_tf.py
  scripts/steer_center_calibration.py
  scripts/wait_for_topics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

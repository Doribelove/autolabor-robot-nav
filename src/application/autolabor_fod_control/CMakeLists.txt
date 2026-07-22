cmake_minimum_required(VERSION 3.0.2)
project(autolabor_fod_control)

find_package(catkin REQUIRED COMPONENTS
  autolabor_canbus_driver
  autolabor_fod_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  autolabor_canbus_driver
  autolabor_fod_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_install_python(PROGRAMS
  scripts/fod_visual_servo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_visual_servo.py)
  find_package(rostest REQUIRED)
  add_rostest(test/visual_servo_integration.test)
endif()

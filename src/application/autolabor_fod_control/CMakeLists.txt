cmake_minimum_required(VERSION 3.0.2)
project(autolabor_fod_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  autolabor_canbus_driver
  autolabor_fod_msgs
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  move_base_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/VisualServo.cfg
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  autolabor_canbus_driver
  autolabor_fod_msgs
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  move_base_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_install_python(PROGRAMS
  scripts/fod_navigation_mode_manager.py
  scripts/fod_visual_servo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  cfg/VisualServo.cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_mode_manager.py)
  catkin_add_nosetests(test/test_visual_servo.py)
  find_package(rostest REQUIRED)
  add_rostest(test/fod_navigation_mode.test)
  add_rostest(test/visual_servo_integration.test)
endif()

cmake_minimum_required(VERSION 3.0.2)
project(autolabor_dual_host)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  autolabor_coverage
  diagnostic_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    autolabor_coverage
    diagnostic_msgs
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
  scripts/cmd_vel_watchdog.py
  scripts/move_base_pause_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/j6m_stack.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_cmd_vel_watchdog.py)
  catkin_add_nosetests(test/test_dual_host_contract.py)
  catkin_add_nosetests(test/test_move_base_pause_bridge.py)
endif()

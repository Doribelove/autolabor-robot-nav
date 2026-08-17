cmake_minimum_required(VERSION 3.0.2)
project(gps_module)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_gps_localization_motion.py)
  catkin_add_nosetests(test/test_gps_long_range_goal.py)
endif()

catkin_install_python(PROGRAMS
  scripts/gps_localization_node.py
  scripts/gps_goal_node.py
  scripts/gps_goal_publisher_node.py
  scripts/gps_long_range_goal_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

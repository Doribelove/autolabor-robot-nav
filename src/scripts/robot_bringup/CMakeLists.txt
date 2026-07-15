cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  base_local_planner
  costmap_2d
  geometry_msgs
  move_base_msgs
  nav_msgs
  nav_core
  pluginlib
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ackermann_arc_recovery
  CATKIN_DEPENDS
    actionlib_msgs
    base_local_planner
    costmap_2d
    geometry_msgs
    move_base_msgs
    nav_msgs
    nav_core
    pluginlib
    roscpp
    rospy
    sensor_msgs
    tf2
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_self_filter src/pointcloud_self_filter.cpp)
target_link_libraries(pointcloud_self_filter ${catkin_LIBRARIES})

add_library(ackermann_arc_recovery src/ackermann_arc_recovery.cpp)
target_link_libraries(ackermann_arc_recovery ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_gps_goal_speed_limiter.py)
  catkin_add_nosetests(test/test_long_range_costmap.py)
  catkin_add_nosetests(test/test_teb_profile_tuning.py)
  catkin_add_gtest(test_ackermann_recovery_geometry
    test/test_ackermann_recovery_geometry.cpp
  )
  if(TARGET test_ackermann_recovery_geometry)
    target_link_libraries(test_ackermann_recovery_geometry
      ackermann_arc_recovery
      ${catkin_LIBRARIES}
    )
  endif()
endif()

catkin_install_python(PROGRAMS
  scripts/autolabor_cmd_test.py
  scripts/autolabor_keyboard_teleop.py
  scripts/gps_goal_speed_limiter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ackermann_recovery_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ackermann_arc_recovery pointcloud_self_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  livox_ros_driver2
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_self_filter src/pointcloud_self_filter.cpp)
target_link_libraries(pointcloud_self_filter ${catkin_LIBRARIES})

add_executable(livox_obstacle_cloud src/livox_obstacle_cloud.cpp)
add_dependencies(livox_obstacle_cloud ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_obstacle_cloud ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/livox_obstacle_scan.test)
  catkin_add_nosetests(test/test_gps_goal_speed_limiter.py)
  catkin_add_nosetests(test/test_teb_profile_tuning.py)
endif()

catkin_install_python(PROGRAMS
  scripts/autolabor_cmd_test.py
  scripts/autolabor_keyboard_teleop.py
  scripts/gps_goal_speed_limiter.py
  scripts/vehicle_outline.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS pointcloud_self_filter livox_obstacle_cloud
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  costmap_2d
  geometry_msgs
  livox_ros_driver2
  message_filters
  move_base_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(roscpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gps_geofence_layer
  CATKIN_DEPENDS costmap_2d livox_ros_driver2 message_filters nav_msgs pluginlib roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(gps_geofence_layer
  src/gps_geofence_layer.cpp
  src/unknown_space_guard_layer.cpp
)
target_link_libraries(gps_geofence_layer ${catkin_LIBRARIES})

add_executable(pointcloud_self_filter src/pointcloud_self_filter.cpp)
target_link_libraries(pointcloud_self_filter ${catkin_LIBRARIES})

add_executable(livox_custom_to_pointcloud src/livox_custom_to_pointcloud.cpp)
add_dependencies(livox_custom_to_pointcloud ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_custom_to_pointcloud ${catkin_LIBRARIES})

add_executable(voxel_cloud_mapper src/voxel_cloud_mapper.cpp)
add_dependencies(voxel_cloud_mapper ${catkin_EXPORTED_TARGETS})
target_include_directories(voxel_cloud_mapper PRIVATE ${roscpp_INCLUDE_DIRS})
target_link_libraries(voxel_cloud_mapper ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/gps_geofence_costmap.test)
  catkin_add_gtest(test_gps_geofence_geometry test/test_gps_geofence_geometry.cpp)
  if(TARGET test_gps_geofence_geometry)
    target_link_libraries(test_gps_geofence_geometry gps_geofence_layer ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_unknown_space_guard test/test_unknown_space_guard.cpp)
  if(TARGET test_unknown_space_guard)
    target_link_libraries(test_unknown_space_guard gps_geofence_layer ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_moving_self_crop test/test_moving_self_crop.cpp)
  catkin_add_nosetests(test/test_gps_geofence_recorder.py)
  catkin_add_nosetests(test/test_gps_goal_speed_limiter.py)
  catkin_add_nosetests(test/test_teb_profile_tuning.py)
  catkin_add_nosetests(test/test_avoidance_scan_fusion_math.py)
  catkin_add_nosetests(test/test_fused_scan_mapper.py)
  catkin_add_nosetests(test/test_map_set_fuser.py)
  catkin_add_nosetests(test/test_localization_cmd_vel_gate.py)
  catkin_add_nosetests(test/test_static_map_navigation.py)
  add_rostest(test/livox_custom_to_pointcloud.test)
  add_rostest(test/avoidance_scan_fusion.test)
endif()

catkin_install_python(PROGRAMS
  scripts/autolabor_cmd_test.py
  scripts/autolabor_keyboard_teleop.py
  scripts/gps_geofence_recorder.py
  scripts/gps_goal_speed_limiter.py
  scripts/fused_scan_mapper.py
  scripts/map_set_fuser.py
  scripts/localization_cmd_vel_gate.py
  scripts/optional_laserscan_fusion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS gps_geofence_layer pointcloud_self_filter livox_custom_to_pointcloud voxel_cloud_mapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

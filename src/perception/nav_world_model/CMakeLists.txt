cmake_minimum_required(VERSION 3.0.2)
project(nav_world_model)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  costmap_converter
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()

add_message_files(FILES
  PredictedState.msg
  TrackedObstacle.msg
  TrackedObstacleArray.msg
  LocalGeometry.msg
  WorldModelHealth.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  costmap_converter
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/nav_world_model_skeleton_node.py
  scripts/nav_world_model_node.py
  scripts/tracks_to_teb_obstacles.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_world_model_core.py)
  catkin_add_nosetests(tests/test_teb_bridge.py)
endif()

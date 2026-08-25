cmake_minimum_required(VERSION 3.0.2)
project(autolabor_coverage)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  autolabor_canbus_driver
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_generation
  move_base_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)

catkin_python_setup()

add_message_files(FILES
  CoverageStatus.msg
  EnforcedPath.msg
)

add_service_files(FILES
  PlanCoverage.srv
  SetEnforcedPath.srv
  StartCoverage.srv
)

generate_messages(DEPENDENCIES geometry_msgs nav_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coverage_global_planner
  CATKIN_DEPENDS
    actionlib autolabor_canbus_driver base_local_planner costmap_2d
    dynamic_reconfigure geometry_msgs message_runtime
    move_base_msgs nav_core nav_msgs navfn pluginlib roscpp rospy sensor_msgs
    std_msgs std_srvs tf2_ros visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(coverage_global_planner src/coverage_global_planner.cpp)
add_dependencies(coverage_global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_global_planner ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/coverage_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS coverage_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES coverage_global_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_coverage_geometry.py)
  catkin_add_nosetests(test/test_coverage_contract.py)
  catkin_add_nosetests(test/test_coverage_manager_state_machine.py)
endif()

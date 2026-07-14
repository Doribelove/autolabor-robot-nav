cmake_minimum_required(VERSION 3.10)
project(m2_gazebo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  autolabor_canbus_driver
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES m2_ackermann_plugin v2_trajectory_actor_plugin
  CATKIN_DEPENDS autolabor_canbus_driver geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros
  DEPENDS GAZEBO
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(m2_ackermann_plugin src/m2_ackermann_plugin.cpp)
add_dependencies(m2_ackermann_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(m2_ackermann_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(v2_trajectory_actor_plugin src/v2_trajectory_actor_plugin.cpp)
add_dependencies(v2_trajectory_actor_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(v2_trajectory_actor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS scripts/m2_regression_test.py scripts/m2_teb_regression_test.py
  scripts/v2_scan_transport.py scripts/v2_dynamics_regression_test.py
  scripts/v2_scene_runtime_probe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS m2_ackermann_plugin v2_trajectory_actor_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_ackermann_kinematics tests/test_ackermann_kinematics.cpp)
  if(TARGET test_ackermann_kinematics)
    target_link_libraries(test_ackermann_kinematics ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(tests/test_model_contract.py)
  add_rostest(tests/v2_dynamics_regression.test)
endif()

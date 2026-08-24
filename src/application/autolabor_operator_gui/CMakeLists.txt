cmake_minimum_required(VERSION 3.0.2)
project(autolabor_operator_gui)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  autolabor_coverage
  autolabor_canbus_driver
  autolabor_fod_msgs
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  rviz
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

find_package(Qt5 REQUIRED COMPONENTS
  Concurrent
  Widgets
)

# MapDisplay exposes its loaded texture dimensions only from RViz's default
# plugin library.  The GUI links it directly so startup can verify that the
# embedded display (not merely a parallel /map subscriber) is actually ready.
find_library(RVIZ_DEFAULT_PLUGIN_LIBRARY
  NAMES rviz_default_plugin
  HINTS ${catkin_LIBRARY_DIRS}
)
if(NOT RVIZ_DEFAULT_PLUGIN_LIBRARY)
  message(FATAL_ERROR "RViz default plugin library is required")
endif()

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    autolabor_coverage
    autolabor_canbus_driver
    autolabor_fod_msgs
    diagnostic_msgs
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    roscpp
    roslib
    rviz
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/main_window.cpp
  include/autolabor_operator_gui/main_window.h
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  Qt5::Concurrent
  Qt5::Widgets
  ${RVIZ_DEFAULT_PLUGIN_LIBRARY}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_operator_gui_contract.py)
endif()

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

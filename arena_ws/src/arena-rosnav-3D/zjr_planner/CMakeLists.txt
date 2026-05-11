cmake_minimum_required(VERSION 3.0.2)
project(zjr_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  GenerateRRTPaths.srv
  SaveScan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(nlohmann_json REQUIRED) # requires libnlohmann-json-dev or vcpkg install

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rrt_multi_paths_srv src/rrt_multi_paths_srv.cpp)
add_dependencies(rrt_multi_paths_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_multi_paths_srv ${catkin_LIBRARIES})

add_executable(save_pointcloud_service src/save_pointcloud_service.cpp)
add_dependencies(save_pointcloud_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(save_pointcloud_service PRIVATE ${nlohmann_json_INCLUDE_DIRS})
target_link_libraries(save_pointcloud_service ${catkin_LIBRARIES} nlohmann_json::nlohmann_json)

catkin_install_python(PROGRAMS
  scripts/logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



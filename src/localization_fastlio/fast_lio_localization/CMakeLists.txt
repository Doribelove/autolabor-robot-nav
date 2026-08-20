cmake_minimum_required(VERSION 3.0.2)
project(fast_lio_localization)

add_compile_options(-std=c++14 -O3 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters io kdtree registration)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp
                 sensor_msgs std_msgs tf2_ros
  DEPENDS PCL
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(fast_lio_map_localizer src/fast_lio_map_localizer.cpp)
target_link_libraries(fast_lio_map_localizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS fast_lio_map_localizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_known_map_localizer_contract.py)
endif()

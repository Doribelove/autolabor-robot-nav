cmake_minimum_required(VERSION 3.0.2)
project(autolabor_dual_lidar)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(optional_cloud_enhancer src/optional_cloud_enhancer.cpp)
target_link_libraries(optional_cloud_enhancer ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS scripts/dual_laser_fusion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_fusion_math.py)
  add_rostest(test/optional_cloud_enhancer.test)
endif()

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS optional_cloud_enhancer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

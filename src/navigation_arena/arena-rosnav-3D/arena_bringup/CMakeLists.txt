cmake_minimum_required(VERSION 3.0.2)
project(arena_bringup)

find_package(catkin REQUIRED)


catkin_package()

include_directories(
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

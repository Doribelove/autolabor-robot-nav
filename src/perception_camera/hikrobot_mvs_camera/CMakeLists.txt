cmake_minimum_required(VERSION 3.0.2)
project(hikrobot_mvs_camera)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  image_transport
  message_generation
  roscpp
  sensor_msgs
)

add_service_files(
  FILES
    GetImagingControls.srv
    SetImagingControls.srv
)

generate_messages()

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS /opt/MVS/include
  NO_DEFAULT_PATH
)

find_library(MVS_CAMERA_CONTROL_LIBRARY
  NAMES MvCameraControl
  PATHS /opt/MVS/lib/64
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_CAMERA_CONTROL_LIBRARY)
  message(FATAL_ERROR
    "Hikrobot MVS SDK was not found. Expected "
    "/opt/MVS/include/MvCameraControl.h and "
    "/opt/MVS/lib/64/libMvCameraControl.so."
  )
endif()

catkin_package(
  CATKIN_DEPENDS
    camera_info_manager
    image_transport
    message_runtime
    roscpp
    sensor_msgs
)

include_directories(
  ${MVS_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hikrobot_mvs_camera_node
  src/hikrobot_mvs_camera_node.cpp
)

add_dependencies(hikrobot_mvs_camera_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hikrobot_mvs_camera_node
  ${catkin_LIBRARIES}
  ${MVS_CAMERA_CONTROL_LIBRARY}
)

set_target_properties(hikrobot_mvs_camera_node PROPERTIES
  BUILD_RPATH "/opt/MVS/lib/64"
  INSTALL_RPATH "/opt/MVS/lib/64"
)

install(TARGETS hikrobot_mvs_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES LICENSE README.md camera_pinhole_my.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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

if(NOT MVS_ROOT AND DEFINED ENV{MVS_ROOT})
  set(MVS_ROOT "$ENV{MVS_ROOT}")
endif()
set(MVS_ROOT "${MVS_ROOT}" CACHE PATH
  "Hikrobot MVS SDK root (contains include/ and lib/)")

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS
    "${MVS_ROOT}/include"
    /opt/MVS/include
  NO_DEFAULT_PATH
)

find_library(MVS_CAMERA_CONTROL_LIBRARY
  NAMES MvCameraControl
  PATHS
    "${MVS_ROOT}/lib/aarch64"
    "${MVS_ROOT}/lib/64"
    "${MVS_ROOT}/lib/x86_64"
    /opt/MVS/lib/aarch64
    /opt/MVS/lib/64
    /opt/MVS/lib/x86_64
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_CAMERA_CONTROL_LIBRARY)
  message(FATAL_ERROR
    "Hikrobot MVS SDK was not found. Set MVS_ROOT to an SDK root containing "
    "include/MvCameraControl.h and an architecture-specific "
    "libMvCameraControl.so (current MVS_ROOT='${MVS_ROOT}')."
  )
endif()
get_filename_component(MVS_LIBRARY_DIR
  "${MVS_CAMERA_CONTROL_LIBRARY}" DIRECTORY)
message(STATUS "Hikrobot MVS include: ${MVS_INCLUDE_DIR}")
message(STATUS "Hikrobot MVS library: ${MVS_CAMERA_CONTROL_LIBRARY}")

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
  BUILD_RPATH "${MVS_LIBRARY_DIR}"
  INSTALL_RPATH "${MVS_LIBRARY_DIR}"
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

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

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK installation")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(MVS_LIBRARY_SUFFIX "aarch64")
else()
  set(MVS_LIBRARY_SUFFIX "64")
endif()

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH
)

find_library(MVS_CAMERA_CONTROL_LIBRARY
  NAMES MvCameraControl
  PATHS "${MVS_ROOT}/lib/${MVS_LIBRARY_SUFFIX}"
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_CAMERA_CONTROL_LIBRARY)
  message(FATAL_ERROR
    "Hikrobot MVS SDK was not found. Expected "
    "${MVS_ROOT}/include/MvCameraControl.h and "
    "${MVS_ROOT}/lib/${MVS_LIBRARY_SUFFIX}/libMvCameraControl.so."
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
target_compile_definitions(hikrobot_mvs_camera_node PRIVATE
  AUTOLABOR_MVS_ROOT="${MVS_ROOT}"
)

add_dependencies(hikrobot_mvs_camera_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hikrobot_mvs_camera_node
  ${catkin_LIBRARIES}
  ${MVS_CAMERA_CONTROL_LIBRARY}
)

get_filename_component(MVS_LIBRARY_DIR "${MVS_CAMERA_CONTROL_LIBRARY}" DIRECTORY)
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

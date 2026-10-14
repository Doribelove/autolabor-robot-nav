cmake_minimum_required(VERSION 3.0.2)
project(autolabor_operator_gui)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  autolabor_canbus_driver
  autolabor_fod_msgs
  autolabor_gps_keepout
  autolabor_operator_msgs
  diagnostic_msgs
  dynamic_reconfigure
  hikrobot_mvs_camera
  nav_msgs
  roscpp
  roslib
  rviz
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(Qt5 REQUIRED COMPONENTS
  Concurrent
  Widgets
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    autolabor_canbus_driver
    autolabor_fod_msgs
    autolabor_gps_keepout
    autolabor_operator_msgs
    diagnostic_msgs
    dynamic_reconfigure
    hikrobot_mvs_camera
    nav_msgs
    roscpp
    roslib
    rviz
    sensor_msgs
    std_msgs
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/main_window.cpp
  src/sweep_panel.cpp
  src/zed_camera_panel.cpp
  src/gps_map_panel.cpp
  include/autolabor_operator_gui/main_window.h
  include/autolabor_operator_gui/sweep_panel.h
  include/autolabor_operator_gui/zed_camera_panel.h
  include/autolabor_operator_gui/gps_map_panel.h
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  Qt5::Concurrent
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_operator_gui_contract.py)
  find_package(Qt5 REQUIRED COMPONENTS Test)
  add_executable(test_gps_map_panel
    test/test_gps_map_panel.cpp
    src/gps_map_panel.cpp
    include/autolabor_operator_gui/gps_map_panel.h
  )
  add_dependencies(test_gps_map_panel ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_gps_map_panel Qt5::Widgets Qt5::Concurrent Qt5::Test ${catkin_LIBRARIES})
  add_test(NAME operator_gps_map_panel COMMAND test_gps_map_panel)
  set_tests_properties(operator_gps_map_panel PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  add_executable(test_sweep_panel
    test/test_sweep_panel.cpp
    src/sweep_panel.cpp
    include/autolabor_operator_gui/sweep_panel.h
  )
  target_compile_definitions(test_sweep_panel PRIVATE TEST_WORKSPACE="${CMAKE_SOURCE_DIR}/..")
  target_link_libraries(test_sweep_panel Qt5::Widgets Qt5::Test)
  add_test(NAME operator_sweep_panel COMMAND test_sweep_panel)
  set_tests_properties(operator_sweep_panel PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  add_executable(test_zed_camera_panel
    test/test_zed_camera_panel.cpp
    src/zed_camera_panel.cpp
    include/autolabor_operator_gui/zed_camera_panel.h
  )
  add_dependencies(test_zed_camera_panel ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_zed_camera_panel Qt5::Widgets Qt5::Concurrent Qt5::Test ${catkin_LIBRARIES})
  find_package(rostest REQUIRED)
  add_rostest(test/zed_camera_panel.test DEPENDENCIES test_zed_camera_panel)
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

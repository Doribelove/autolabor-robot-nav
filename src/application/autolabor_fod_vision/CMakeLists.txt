cmake_minimum_required(VERSION 3.0.2)
project(autolabor_fod_vision)

find_package(catkin REQUIRED COMPONENTS
  autolabor_fod_msgs
  cv_bridge
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  autolabor_fod_msgs
  cv_bridge
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/fod_detect_and_classify_node.py
  scripts/fod_detector_node.py
  scripts/fod_ground_projector_node.py
  scripts/fod_image_quality_controller_node.py
  scripts/fod_media_source_node.py
  scripts/fod_sim_source_node.py
  scripts/fod_tracker_node.py
  scripts/fod_vision_result_adapter_node.py
  scripts/yolo_smoke_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_camera_info.py)
  catkin_add_nosetests(test/test_clip_filter.py)
  catkin_add_nosetests(test/test_confidence_control.py)
  catkin_add_nosetests(test/test_depth_fusion.py)
  catkin_add_nosetests(test/test_image_quality.py)
  catkin_add_nosetests(test/test_legacy_display_depth_contract.py)
  catkin_add_nosetests(test/test_locateanything_runtime.py)
  catkin_add_nosetests(test/test_projection.py)
  catkin_add_nosetests(test/test_tracking.py)
  catkin_add_nosetests(test/test_two_stage.py)
  catkin_add_nosetests(test/test_two_stage_runtime_contract.py)
  catkin_add_nosetests(test/test_ultralytics_runtime.py)
endif()

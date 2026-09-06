cmake_minimum_required(VERSION 3.0.2)
project(autolabor_bpu_perception)
find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs sensor_msgs
  geometry_msgs visualization_msgs nav_msgs tf2_ros livox_ros_driver2 costmap_converter)
catkin_python_setup()
add_message_files(FILES DetectedObject.msg DetectedObjects.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs geometry_msgs
  visualization_msgs nav_msgs tf2_ros livox_ros_driver2 costmap_converter)
catkin_install_python(PROGRAMS scripts/centerpoint_node.py scripts/obstacle_adapter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_core.py)
  catkin_add_nosetests(test/test_centerpoint.py)
  catkin_add_nosetests(test/test_protocol.py)
  catkin_add_nosetests(test/test_obstacle_adapter.py)
endif()

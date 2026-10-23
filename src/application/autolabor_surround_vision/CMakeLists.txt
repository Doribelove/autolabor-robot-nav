cmake_minimum_required(VERSION 3.0.2)
project(autolabor_surround_vision)
find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs message_generation)
add_message_files(FILES
  SurroundSource.msg SurroundObservation.msg SurroundObservationArray.msg
  SurroundTarget.msg SurroundTargetArray.msg)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs)
catkin_install_python(PROGRAMS scripts/surround_bridge.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

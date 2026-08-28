cmake_minimum_required(VERSION 3.0.2)
project(sweeper_mcp)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
  autolabor_coverage
  move_base_msgs
  message_generation
  tf2_ros
)

# Install the Python package before declaring generated interfaces; catkin
# requires this ordering when both mechanisms are used by one package.
catkin_python_setup()

add_message_files(FILES
  AiControlStatus.msg
  AiEvent.msg
)

add_service_files(FILES
  SetAiAuthorization.srv
  SetAsrModel.srv
  SetAsrRecording.srv
  SetSmartVoice.srv
  SubmitAiText.srv
  CancelAiTask.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib rospy std_msgs std_srvs geometry_msgs sensor_msgs
    nav_msgs actionlib_msgs autolabor_coverage move_base_msgs message_runtime tf2_ros
)

# ---- 安装规则 ----
catkin_install_python(PROGRAMS
  scripts/mcp_sweeper_server.py
  scripts/sweeper_ai_node.py
  scripts/whisper_asr_worker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 shell 脚本
install(PROGRAMS
  scripts/start_mcp_server.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "sweeper_mcp.yaml" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(scripts/test_mcp_protocol.py)
  catkin_add_nosetests(scripts/test_agent_planning.py)
  catkin_add_nosetests(scripts/test_asr_authorization.py)
  catkin_add_nosetests(scripts/test_asr_worker.py)
  catkin_add_nosetests(scripts/test_ros_backend_map_navigation.py)
  catkin_add_nosetests(scripts/test_ros_backend_goal_ownership.py)
endif()

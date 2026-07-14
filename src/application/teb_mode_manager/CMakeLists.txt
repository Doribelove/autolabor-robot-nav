cmake_minimum_required(VERSION 3.0.2)
project(teb_mode_manager)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  nav_world_model
  rosgraph_msgs
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(FILES
  ContextState.msg
  ModeTransition.msg
  ParameterTransaction.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  dynamic_reconfigure
  message_runtime
  nav_world_model
  rosgraph_msgs
  rospy
  std_msgs
)

catkin_install_python(PROGRAMS
  scripts/teb_mode_manager_skeleton_node.py
  scripts/rule_context_supervisor_node.py
  scripts/rule_anchor_transaction_node.py
  scripts/simulation_typed_anchor_transaction_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_rule_supervisor.py)
  catkin_add_nosetests(tests/test_action_pipeline.py)
  catkin_add_nosetests(tests/test_typed_teb_transaction.py)
endif()

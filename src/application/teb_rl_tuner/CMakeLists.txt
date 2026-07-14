cmake_minimum_required(VERSION 3.0.2)
project(teb_rl_tuner)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS dynamic_reconfigure rospy)

catkin_install_python(PROGRAMS
  scripts/t03_teb_client_integration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(tests/test_config_validation.py)
  catkin_add_nosetests(tests/test_teb_parameter_client.py)
  catkin_add_nosetests(tests/test_t04_timing.py)
  catkin_add_nosetests(tests/test_t04_state_builder.py)
  catkin_add_nosetests(tests/test_t04_reward_cost.py)
  catkin_add_nosetests(tests/test_t04_episode_state_machine.py)
  catkin_add_nosetests(tests/test_t05_parameter_projection.py)
  catkin_add_nosetests(tests/test_t05_safety_fallback.py)
  catkin_add_nosetests(tests/test_training_environment.py)
  catkin_add_nosetests(tests/test_baseline_policy.py)
  add_rostest(tests/t03_teb_client_integration.test)
endif()

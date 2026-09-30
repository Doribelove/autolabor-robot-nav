cmake_minimum_required(VERSION 3.0.2)
project(v4_scope_rl)

catkin_python_setup()
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/v4_audit_learning_value.py
  scripts/v4_train_masked_ppo.py
  scripts/v4_evaluate_policy.py
  scripts/v4_export_thesis_resources.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

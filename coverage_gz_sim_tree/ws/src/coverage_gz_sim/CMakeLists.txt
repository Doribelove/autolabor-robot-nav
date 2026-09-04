cmake_minimum_required(VERSION 3.0.2)
project(coverage_gz_sim)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/m2_truth_sim.py
  scripts/sim_support.py
  scripts/start_batch.py
  scripts/coverage_monitor.py
  scripts/audit_navigation_bag.py
  scripts/analyze_navigation_details.py
  scripts/analyze_teb_plan_deviation.py
  scripts/check_transition_budget.py
  scripts/navigation_fault_injector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_m2_truth_sim.py)
  catkin_add_nosetests(test/test_navigation_fault_injector.py)
endif()

install(DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

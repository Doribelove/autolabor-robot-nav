cmake_minimum_required(VERSION 3.0.2)
project(v3_scope_planner)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(FILES
  TrajectoryPoint.msg
  CandidateTrajectory.msg
  CandidateSet.msg
  CandidateEvaluation.msg
  CandidateDecision.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)

catkin_install_python(PROGRAMS
  scripts/v3_scope_planner_offline_probe.py
  scripts/v3_scope_actual_teb_fair_episode.py
  scripts/v3_scope_thesis_benchmark_episode.py
  scripts/v3_scope_multiactor_clone_episode.py
  scripts/v3_scope_multiactor_sequential_clone_episode.py
  scripts/v3_scope_multiactor_sequential_clone_episode_v2.py
  scripts/v3_scope_multiactor_actual_teb_fair_episode.py
  scripts/v3_scope_multiactor_actual_teb_diagnostic_episode.py
  scripts/v3_scope_multiactor_midpoint_clone_episode.py
  scripts/v3_scope_recovery_candidate_preflight_episode.py
  scripts/v3_scope_progress_recovery_preflight_episode.py
  scripts/v3_scope_progress_recovery_bank_clone_episode.py
  scripts/v3_scope_progress_predictor_qualification_episode.py
  scripts/v3_scope_progress_actual_teb_preflight_episode.py
  scripts/v3_scope_goal_aligned_recovery_preflight_episode.py
  scripts/v3_scope_goal_aligned_recovery_preflight_episode_r1.py
  scripts/v3_scope_goal_aligned_recovery_bank_clone_episode.py
  scripts/v3_scope_goal_aligned_boundary_atomic_preflight_episode.py
  scripts/v3_scope_boundary_atomic_recovery_bank_clone_episode.py
  scripts/v3_scope_recovery_bank_clone_episode.py
  scripts/v3_scope_recovery_predictor_qualification_episode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

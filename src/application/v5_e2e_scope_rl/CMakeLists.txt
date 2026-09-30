cmake_minimum_required(VERSION 3.0.2)
project(v5_e2e_scope_rl)

catkin_python_setup()
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/v6_sensor_gazebo_episode.py
  scripts/v6_sensor_gazebo_episode_r2.py
  scripts/v6_sensor_gazebo_episode_v604.py
  scripts/pct_teb_dwa_benchmark_episode.py
  scripts/pct_teb_dwa_benchmark_r2_episode.py
  scripts/arena_pct_teb_dwa_qualification_episode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

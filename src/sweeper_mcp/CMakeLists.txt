cmake_minimum_required(VERSION 3.0.2)
project(sweeper_mcp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
)

# 启用 catkin Python 包安装: 将 src/sweeper_mcp 作为 Python 包安装到 devel
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs
)

# ---- 安装规则 ----
catkin_install_python(PROGRAMS
  scripts/mcp_sweeper_server.py
  scripts/test_mcp_protocol.py
  scripts/test_deepseek_tools.py
  scripts/test_sim_voice.py
  scripts/voice_agent_cli.py
  scripts/voice_agent_sim.py
  scripts/voice_agent_node.py
  scripts/keyboard_input.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 shell 脚本
install(PROGRAMS
  scripts/start_mcp_server.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#!/bin/bash
# ============================================================================
# 启动 sweeper_mcp 的 MCP stdio server（供外部 MCP host 调用，如 Claude Code /
# Claude Desktop / 任意 MCP 客户端）。
#
# MCP host 侧注册命令填（Windows 换 cmd.exe 路径）：
#   bash -c 'source /opt/ros/noetic/setup.bash \
#          && source /mnt/storage/robot_project/autorobot/autolabor-robot-nav/devel/setup.bash \
#          && exec /mnt/storage/robot_project/autorobot/autolabor-robot-nav/src/sweeper_mcp/scripts/start_mcp_server.sh'
#
# 参数:
#   mock        离线模式（MCP_BACKEND=mock，不碰 ROS，用于联调协议/工具 schema）
#   ros         真实 ROS（默认，需先 source ROS 环境 & 起仿真/真车）
#   --no-source 不 source 环境（已 source 过时用）
#
# 说明:
#   - server 要 import rospy、连 ROS master，所以必须先 source ROS 环境；
#   - 运行期间在终端逐行收发 JSON-RPC（stdio），别在终端输入其他内容；
#   - 更多连调方式见 ../README.md 第 4.7 节。
# ============================================================================
set -u

MODE="${1:-ros}"
PROJ_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SERVER="$PROJ_ROOT/src/sweeper_mcp/scripts/mcp_sweeper_server.py"

case "$MODE" in
    mock) export MCP_BACKEND="mock" ;;
    ros)  export MCP_BACKEND="ros" ;;
    --no-source)
        MODE="ros"; export MCP_BACKEND="ros"
        exec python3 "$SERVER"; exit 0
        ;;
    *) echo "未知模式: $MODE（mock / ros）" >&2; exit 1 ;;
esac

if ! command -v roscore >/dev/null 2>&1; then
    source /opt/ros/noetic/setup.bash || { echo "找不到 ROS，请先 source 环境" >&2; exit 1; }
fi
if [ ! -f "$PROJ_ROOT/devel/setup.bash" ]; then
    echo "未找到 $PROJ_ROOT/devel/setup.bash，请先构建工作区" >&2; exit 1
fi
source "$PROJ_ROOT/devel/setup.bash"

exec python3 "$SERVER"

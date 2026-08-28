from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# CUDA Whisper 及 large-v3 权重安装在工作区 runtime/asr 的独立虚拟环境；
# catkin Python 包只安装 ASR 子进程客户端、授权、规划和 MCP 控制代码。
d = generate_distutils_setup(
    packages=["sweeper_mcp"],
    package_dir={"": "src"},
)

setup(**d)

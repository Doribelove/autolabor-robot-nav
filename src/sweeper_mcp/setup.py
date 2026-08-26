from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# 把 src/sweeper_mcp 作为 Python 包安装到 devel 命名空间。
# voice 子模块（AI 语音：asr_audio / asr_recognizer）一并安装。
d = generate_distutils_setup(
    packages=["sweeper_mcp", "sweeper_mcp.voice"],
    package_dir={"": "src"},
)

setup(**d)

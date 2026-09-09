FROM ros:noetic-ros-base-focal
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-nose python3-yaml python3-rosdep \
    ros-noetic-move-base ros-noetic-navfn ros-noetic-teb-local-planner \
    ros-noetic-tf2-geometry-msgs ros-noetic-tf2-eigen \
    libgtest-dev libeigen3-dev libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /coverage_ws
COPY . .
RUN bash tools/build.sh
CMD ["bash", "tools/test.sh"]

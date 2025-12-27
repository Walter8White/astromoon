# docker/Dockerfile
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Basics + build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    ca-certificates \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    x11-apps \
    x11-xserver-utils \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Fortress
RUN apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# ROS <-> Gazebo bridge (try both names depending on apt repos availability)
RUN apt-get update && \
    (apt-get install -y --no-install-recommends ros-humble-ros-gz || true) && \
    (apt-get install -y --no-install-recommends ros-humble-ros-ign || true) && \
    rm -rf /var/lib/apt/lists/*

# rosdep init/update (safe in container build)
RUN rosdep init || true
RUN rosdep update

# Workspace path (code will be bind-mounted here by docker-compose)
ENV WS=/astromoon_ws
WORKDIR ${WS}

# Shell convenience: source ROS by default; source workspace if built
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo 'if [ -f /astromoon_ws/install/setup.bash ]; then source /astromoon_ws/install/setup.bash; fi' >> /etc/bash.bashrc && \
    echo "cd /astromoon_ws" >> /etc/bash.bashrc

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM osrf/ros:humble-desktop


ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Basics
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Fortress + ROS<->Gazebo bridge (nom du package dépend des dépôts)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    && (apt-get install -y --no-install-recommends ros-humble-ros-gz || true) \
    && (apt-get install -y --no-install-recommends ros-humble-ros-ign || true) \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# rosdep init (ok si déjà initialisé)
RUN rosdep init || true
RUN rosdep update

# Workspace
ENV WS=/astromoon_ws
WORKDIR ${WS}

# Copie du workspace (on suppose que ton repo contient astromoon_ws/*)
# Si ton repo EST le workspace, adapte (on ajustera si besoin)
COPY . ${WS}

# Déps + build (ignore les sources non résolues si jamais)
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "if [ -f /astromoon_ws/install/setup.bash ]; then source /astromoon_ws/install/setup.bash; fi" >> /etc/bash.bashrc && \
    echo "cd /astromoon_ws" >> /etc/bash.bashrc
# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

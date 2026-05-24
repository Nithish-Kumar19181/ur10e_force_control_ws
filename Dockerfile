FROM osrf/ros:humble-desktop-full

# ── System utilities ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    vim \
    sudo \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 packages ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Universal Robots driver + controllers
    ros-humble-ur-robot-driver \
    ros-humble-ur-controllers \
    ros-humble-ur-moveit-config \
    ros-humble-ur-description \
    # MoveIt2
    ros-humble-moveit \
    # Gazebo Classic (11) + ROS2 bridge
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    # Ignition/GZ Fortress + ROS2 bridge
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ign-ros2-control \
    # ros2_control stack
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    # Cartesian controller build dependencies
    ros-humble-eigen-stl-containers \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Non-root user (matches host UID/GID to avoid permission issues) ───────────
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && mkdir -p /home/${USERNAME}/.config \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ── Workspace mount point ─────────────────────────────────────────────────────
RUN mkdir -p /workspace && chown ${USER_UID}:${USER_GID} /workspace

WORKDIR /workspace

# ── Shell setup for ros user ──────────────────────────────────────────────────
USER ${USERNAME}

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]

ARG ROS_DISTRO=humble

# Use base image
FROM ros:$ROS_DISTRO-ros-base

# Re-declare after FROM so $ROS_DISTRO is available in subsequent instructions
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive

# ── System utilities ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    nano \
    vim \
    curl \
    wget \
    git \
    tmux \
    xterm \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pyqt5 \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 packages ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ur-robot-driver \
    ros-$ROS_DISTRO-ur-controllers \
    ros-$ROS_DISTRO-ur-moveit-config \
    ros-$ROS_DISTRO-ur-description \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-eigen-stl-containers \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace ─────────────────────────────────────────────────────────────────
ENV WORKSPACE_DIR=/ur10e_ws
RUN mkdir -p $WORKSPACE_DIR/src

WORKDIR $WORKSPACE_DIR

# Initialize rosdep
RUN rosdep init || true \
    && rosdep update --rosdistro $ROS_DISTRO

ENV ROS_DOMAIN_ID=0

# Copy workspace source into image (submodules must be initialized before docker build)
COPY workspace/ $WORKSPACE_DIR/src/

# Install ROS dependencies declared in package.xml files
RUN apt-get update \
    && rosdep install --from-paths $WORKSPACE_DIR/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build workspace — skip test and simulation stubs from cartesian_controllers
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install \
      --packages-skip cartesian_controller_simulation cartesian_controller_tests \
      --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Source ROS and workspace on every shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source $WORKSPACE_DIR/install/setup.bash" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc

CMD ["bash"]

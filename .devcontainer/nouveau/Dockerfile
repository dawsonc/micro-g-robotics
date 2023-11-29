FROM ghcr.io/dawsonc/micro-g-robotics:humble-base

# Install RealSense SDK
RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y apt-transport-https \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" \
    | sudo tee /etc/apt/sources.list.d/librealsense.list \
    && sudo apt-get update -q \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg \
    && sudo apt install -y ros-humble-realsense2-*

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
ENV USERNAME=buzz
ENV USER_WORKSPACE=/home/$USERNAME/ws_micro_g
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/micro-g-robotics

# Clone AprilTag node
WORKDIR $USER_WORKSPACE/src
RUN git clone https://github.com/christianrauch/apriltag_ros.git --branch 3.1.2

RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths . --ignore-src -r --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR $USER_WORKSPACE
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build --packages-select apriltag_ros

# Install debugging/linting Python packages
COPY --chown=$USER_UID:$USER_GID requirements-dev.txt .
RUN python3 -m pip install -r requirements-dev.txt \
    && rm -rf requirements-dev.txt

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"

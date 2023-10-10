FROM ghcr.io/dawsonc/micro-g-robotics:humble-base

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
ENV USERNAME=buzz
ENV USER_WORKSPACE=/home/$USERNAME/ws_micro_g
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/micro-g-robotics

RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths . --ignore-src -r --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install NVIDIA software
RUN sudo apt-get update \
    && sudo apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

# Install debugging/linting Python packages
COPY --chown=$USER_UID:$USER_GID requirements-dev.txt .
RUN python3 -m pip install -r requirements-dev.txt \
    && rm -rf requirements-dev.txt

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Include the current workspace in the Gazebo environment search path
ENV IGN_GAZEBO_RESOURCE_PATH=$WORKSPACE/install/micro_g_description/share/micro_g_description/gazebo/worlds:$IGN_GAZEBO_RESOURCE_PATH

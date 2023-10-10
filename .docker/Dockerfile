ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop as ci

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/ws_micro_g

COPY . src/micro-g-robotics

# Install apt packages
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    git \
    sudo \
    clang \
    clang-format-14 \
    clang-tidy \
    clang-tools \
    python3-pip \
    python3-dev \
    apt-utils \
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install all ROS dependencies needed for CI
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

FROM ci as base

# Configure a new non-root user
ARG USERNAME=buzz
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to the non-root user
# The interbotix installation requires us to use the non-root user
USER $USERNAME
ENV USER=$USERNAME

# Download and install external repos used by the project
ENV DEBIAN_FRONTEND=noninteractive
ENV USER_WORKSPACE=/home/$USERNAME/ws_micro_g
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/micro-g-robotics
RUN vcs import src < src/micro-g-robotics/micro_g.repos

# Install all ROS dependencies
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src -r --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install interbotix interfaces
RUN /bin/bash $USER_WORKSPACE/src/interbotix_ros_manipulators/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh -d humble -p $USER_WORKSPACE -n

# Setup the ROS environment
ENV IGN_GAZEBO_RESOURCE_PATH=$USER_WORKSPACE/install/micro_g_description/gazebo/worlds:$IGN_GAZEBO_RESOURCE_PATH
ENV IGN_GAZEBO_RESOURCE_PATH=$USER_WORKSPACE/install/interbotix_xsarm_descriptions/share/interbotix_xsarm_descriptions/meshes:$IGN_GAZEBO_RESOURCE_PATH

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && sudo sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$USER_WORKSPACE/setup.sh#g" /ros_entrypoint.sh \
    && echo "source ${USER_WORKSPACE}/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

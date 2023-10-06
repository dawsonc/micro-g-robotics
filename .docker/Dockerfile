ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base as ci

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/ws_micro_g_robotics

COPY . src/micro_g_robotics

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
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install the Python requirements that aren't available as rosdeps
RUN python3 -m pip install -r $(pwd)/src/micro_g_robotics/requirements-build.txt

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

FROM base as simulation

FROM simulation as desktop

# WARNING: This is a temporary solution for disabling the setuputils installation warning
ENV PYTHONWARNINGS="ignore"

ARG WORKSPACE
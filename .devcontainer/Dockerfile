FROM osrf/ros:humble-desktop-full

#TODO: Add System deps here for anything you add to this repo
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-pip python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
      libeigen3-dev libboost-all-dev libceres-dev libopencv-dev python3-opencv \
      ros-humble-rviz2 ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pymavlink

ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ros 2>/dev/null || groupmod -g ${USER_GID} $(getent group ${USER_GID} | cut -d: -f1) \
 && useradd  --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ros 2>/dev/null \
        || usermod -u ${USER_UID} -g ${USER_GID} ros \
 && apt-get update && apt-get install -y --no-install-recommends sudo \
 && echo "ros ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/ros \
 && chmod 0440 /etc/sudoers.d/ros \
 && rm -rf /var/lib/apt/lists/*

USER ros
WORKDIR /workspaces/anduril_ws
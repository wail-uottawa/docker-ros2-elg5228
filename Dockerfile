# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# This Dockerfile is based on
# https://github.com/AtsushiSaito/docker-ubuntu-sweb
# and
# https://github.com/Tiryoh/docker-ros-desktop-vnc
# which are released under the Apache-2.0 license.

# FROM ubuntu:jammy-20240227 as stage-original
FROM osrf/ros:humble-desktop-full as stage-original

LABEL maintainer "Wail Gueaieb"
MAINTAINER Wail Gueaieb "https://github.com/wail-uottawa/docker-ros2-elg5228"
ENV REFRESHED_AT 2024-05-03

ARG TARGETPLATFORM

SHELL ["/bin/bash", "-c"]

######################################################

RUN apt-get update && apt-get install -y \
    dos2unix \
    curl \
    libglu1-mesa-dev \
    nano \
    evince \
    viewnior \
    filezilla \
    ruby-dev \
    tmux \
    wget \
    xorg-dev \
    zsh

# Upgrade OS
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Install Ubuntu Mate desktop
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ubuntu-mate-desktop && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Add Package
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        tigervnc-standalone-server tigervnc-common \
        supervisor wget curl gosu git python3-pip tini \
        build-essential vim sudo lsb-release locales \
        bash-completion tzdata terminator && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# noVNC and Websockify
RUN git clone https://github.com/AtsushiSaito/noVNC.git -b add_clipboard_support /usr/lib/novnc
RUN pip install git+https://github.com/novnc/websockify.git@v0.10.0
RUN ln -s /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

# Set remote resize function enabled by default
RUN sed -i "s/UI.initSetting('resize', 'off');/UI.initSetting('resize', 'remote');/g" /usr/lib/novnc/app/ui.js

# Disable auto update and crash report
RUN sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades
RUN sed -i 's/enabled=1/enabled=0/g' /etc/default/apport

# Install Firefox
RUN DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:mozillateam/ppa -y && \
    echo 'Package: *' > /etc/apt/preferences.d/mozilla-firefox && \
    echo 'Pin: release o=LP-PPA-mozillateam' >> /etc/apt/preferences.d/mozilla-firefox && \
    echo 'Pin-Priority: 1001' >> /etc/apt/preferences.d/mozilla-firefox && \
    apt-get update -q && \
    apt-get install -y \
    firefox && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Install VSCodium
RUN wget https://gitlab.com/paulcarroty/vscodium-deb-rpm-repo/raw/master/pub.gpg \
    -O /usr/share/keyrings/vscodium-archive-keyring.asc && \
    echo 'deb [ signed-by=/usr/share/keyrings/vscodium-archive-keyring.asc ] https://paulcarroty.gitlab.io/vscodium-deb-rpm-repo/debs vscodium main' \
    | tee /etc/apt/sources.list.d/vscodium.list && \
    apt-get update -q && \
    apt-get install -y codium && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*


#####################################################################
# Add a few ROS packages
FROM stage-original as stage-extra-ros2-packages

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-plotjuggler-ros \
    ros-humble-robot-state-publisher \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rqt-tf-tree \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-twist-mux \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-control-msgs \
    ros-humble-angles \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-moveit \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ur \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot4-simulator


#####################################################################
# Setup Workspace
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
FROM stage-extra-ros2-packages as stage-workspace

ENV ROS_DISTRO humble
# desktop or ros-base
ARG INSTALL_PACKAGE=desktop

# ARG user=ubuntu
ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV ROS2_WS=$HOME/ros2_ws
#WORKDIR $HOME

RUN mkdir -p $ROS2_WS/src && \
    cd $ROS2_WS && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install


#####################################################################
# Install Turtlebot3
# https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
FROM stage-workspace as stage-turtlebot3

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV TURTLEBOT3_WS=$HOME/turtlebot3_ws

RUN mkdir -p $TURTLEBOT3_WS/src && \
    cd $TURTLEBOT3_WS && \
    git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install


#####################################################################
# Install Turtlebot4
# https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
FROM stage-turtlebot3 as stage-turtlebot4

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV TURTLEBOT4_WS=$HOME/turtlebot4_ws

RUN mkdir -p $TURTLEBOT4_WS/src && \
    cd $TURTLEBOT4_WS && \
    git clone -b humble  https://github.com/turtlebot/turtlebot4_simulator.git  src && \
    rosdep update && rosdep install --from-path src -yi && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install


#####################################################################
# Install Husarion robots
#
# ROSbot 2R and ROSbot 2 PRO: https://github.com/husarion/rosbot_ros
# ROSbot XL: https://github.com/husarion/rosbot_xl_ros
# Panther: https://github.com/husarion/panther_ros/tree/ros2

FROM stage-turtlebot4 as stage-husarion_rosbotxl

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV ROSBOTXL_WS=$HOME/rosbotxl_ws

RUN mkdir -p $ROSBOTXL_WS/src && \
    cd $ROSBOTXL_WS && \
    git clone https://github.com/husarion/rosbot_xl_ros src/ && \
    export HUSARION_ROS_BUILD=simulation && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos && \
    vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos && \
    sed '/if(BUILD_TESTING)/,/endif()/d' src/micro_ros_msgs/CMakeLists.txt -i && \
    cp -r src/ros2_controllers/diff_drive_controller src/ && \
    cp -r src/ros2_controllers/imu_sensor_broadcaster src/ && \
    rm -rf src/ros2_controllers && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build


#####################################################################
# Install Husarion robots
#
# ROSbot 2R and ROSbot 2 PRO: https://github.com/husarion/rosbot_ros
# ROSbot XL: https://github.com/husarion/rosbot_xl_ros
# Panther: https://github.com/husarion/panther_ros/tree/ros2

FROM stage-husarion_rosbotxl as stage-husarion_rosbotpr2

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV ROSBOTPR2_WS=$HOME/rosbotpr2_ws

RUN mkdir -p $ROSBOTPR2_WS/src && \
    cd $ROSBOTPR2_WS && \
    git clone https://github.com/husarion/rosbot_ros src/ && \
    export HUSARION_ROS_BUILD=simulation && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    vcs import src < src/rosbot/rosbot_hardware.repos && \
    vcs import src < src/rosbot/rosbot_simulation.repos && \
    cp -r src/ros2_controllers/diff_drive_controller src && cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release



#####################################################################
# Install Universal Robots
# https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble
# https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator
#
# It has already been install above (apt install ros-humble-ur)
# e.g.: ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
# e.g.: ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
# Allowed ur_type values: ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30

FROM stage-husarion_rosbotpr2 as stage-ur


#####################################################################
# Finalization

FROM stage-ur as stage-finalization

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Enable apt-get completion after running `apt-get update` in the container
RUN rm /etc/apt/apt.conf.d/docker-clean


COPY ./entrypoint.sh /
RUN dos2unix /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "-c", "/entrypoint.sh" ]

ENV USER ubuntu
ENV PASSWD ubuntu

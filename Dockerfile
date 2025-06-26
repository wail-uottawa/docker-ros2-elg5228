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
FROM osrf/ros:humble-desktop-full AS stage-original

LABEL maintainer "Wail Gueaieb"
MAINTAINER Wail Gueaieb "https://github.com/wail-uottawa/docker-ros2-elg5228"

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
    zsh \
    iputils-ping

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


# A few tools
RUN apt-get update -q && \
    apt-get install -y \
    featherpad \
    doublecmd-qt


#####################################################################
# Add a few ROS packages
FROM stage-original AS stage-extra-ros2-packages

ENV ROS_DISTRO humble

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-rosbag2-storage-default-plugins \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-turtlebot3 \
    ros-$ROS_DISTRO-turtlebot3-msgs \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-generate-parameter-library \
    ros-$ROS_DISTRO-ros2-control-test-assets \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-angles \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-rqt-robot-steering \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-ur \
    ros-$ROS_DISTRO-turtlebot4-desktop \
    ros-$ROS_DISTRO-turtlebot4-simulator \
    ros-dev-tools \
    python3-vcstool \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-clean
# ros-humble-plotjuggler-ros is temporarily unavailable. Add when available
# More info at https://github.com/facontidavide/PlotJuggler/issues/1074


#####################################################################
# Setup Workspace
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
FROM stage-extra-ros2-packages AS stage-workspace

# ENV ROS_DISTRO humble
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
FROM stage-workspace AS stage-turtlebot3

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV TURTLEBOT3_WS=$HOME/turtlebot3_ws

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-* \
    ros-$ROS_DISTRO-cartographer \
    ros-$ROS_DISTRO-cartographer-ros \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

RUN mkdir -p $TURTLEBOT3_WS/src && \
    cd $TURTLEBOT3_WS/src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    cd $TURTLEBOT3_WS && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install


#####################################################################
# Install Turtlebot4
# https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
FROM stage-turtlebot3 AS stage-turtlebot4

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV TURTLEBOT4_WS=$HOME/turtlebot4_ws

RUN mkdir -p $TURTLEBOT4_WS/src && \
    cd $TURTLEBOT4_WS/src && \
    git clone -b humble  https://github.com/turtlebot/turtlebot4_simulator.git && \
    cd $TURTLEBOT4_WS && \
    rosdep update && rosdep install --from-path src -yi && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install


#####################################################################
# Install Husarion robots
#
# ROSbot 2R and ROSbot 2 PRO: https://github.com/husarion/rosbot_ros
# ROSbot XL: https://github.com/husarion/rosbot_xl_ros
# Panther: https://github.com/husarion/panther_ros/tree/ros2

# FROM stage-turtlebot4 AS stage-husarion_rosbotxl

# ENV USER=ubuntu
# ENV HOME=/home/ubuntu
# ENV ROSBOTXL_WS=$HOME/rosbotxl_ws

# RUN mkdir -p $ROSBOTXL_WS/src && \
#     cd $ROSBOTXL_WS && \
#     git clone https://github.com/husarion/rosbot_xl_ros src/ && \
#     export HUSARION_ROS_BUILD=simulation && \
#     . /opt/ros/$ROS_DISTRO/setup.sh && \
#     vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos && \
#     vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos && \
#     cp -r src/ros2_controllers/diff_drive_controller src/ && \
#     cp -r src/ros2_controllers/imu_sensor_broadcaster src/ && \
#     rm -rf src/ros2_controllers && \
#     rosdep update --rosdistro $ROS_DISTRO && \
#     rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
#     . /opt/ros/$ROS_DISTRO/setup.sh && \
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


#####################################################################
# Install Husarion robots (Rosbot 3, Rosbot 3 Pro and Rosbot XL)
#
# https://husarion.com/
# https://github.com/husarion/rosbot_ros

FROM stage-turtlebot4 AS stage-husarion_rosbot

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV ROSBOT_WS=$HOME/rosbot_ws

RUN apt-get update && apt-get install -y \
    stm32flash

RUN mkdir $ROSBOT_WS && \
    cd $ROSBOT_WS && \
    git clone -b humble https://github.com/husarion/rosbot_ros.git src/rosbot_ros && \
    export HUSARION_ROS_BUILD_TYPE=simulation && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    vcs import src < src/rosbot_ros/rosbot/rosbot_${HUSARION_ROS_BUILD_TYPE}.repos && \
    vcs import src < src/rosbot_ros/rosbot/manipulator.repos && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
# sometimes colcon build needs to be applied multiple times

#####################################################################
# Install Universal Robots
# https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble
# https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator
#
# It has already been install above (apt install ros-humble-ur)
# e.g.: ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
# e.g.: ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
# Allowed ur_type values: ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30

FROM stage-husarion_rosbot AS stage-ur


#####################################################################
# Install TIAGo
#
# https://pal-robotics.com
# https://github.com/pal-robotics/tiago_simulation
# Better work with rmw_cyclonedds_cpp ==> "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"

FROM stage-ur AS stage-tiago

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV TIAGO_WS=$HOME/tiago_ws

RUN mkdir -p $TIAGO_WS/src && \
    cd $TIAGO_WS && \
    vcs import --input https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/humble-devel/tiago_public.repos src && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install


#####################################################################
# Install Franka
#
# https://franka.de
# https://github.com/frankarobotics/franka_ros2
# https://github.com/frankarobotics/franka_ros2/blob/humble/franka_gazebo/README.md

FROM stage-tiago AS stage-franka

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV FRANKA_WS=$HOME/franka_ros2_ws

RUN mkdir -p $FRANKA_WS/src && \
    cd $FRANKA_WS && \
    git clone -b humble https://github.com/frankaemika/franka_ros2.git src && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    vcs import src < src/franka.repos --recursive --skip-existing && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    . $FRANKA_WS/install/setup.bash && \
    colcon build --packages-select franka_example_controllers franka_ign_ros2_control && \
    colcon build --packages-select franka_example_controllers franka_ign_ros2_control && \
    colcon build --packages-select franka_example_controllers franka_ign_ros2_control


#####################################################################
# Install Neobotix
#
# https://neobotix-docs.de/ros/
# https://github.com/neobotix/neo_simulation2

FROM stage-franka AS stage-neobotix

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV NEOBOTIX_TMP=$HOME/tmp

RUN mkdir $NEOBOTIX_TMP && \
    cd $NEOBOTIX_TMP && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    git clone --branch jammy https://github.com/neobotix/robot-setup-tool.git && \
    cd robot-setup-tool/package-setup && \
    ./setup-rox-simulation.sh && \
    ./setup-simulation.sh && \
    cd $HOME && \
    rm -fr $NEOBOTIX_TMP

# for Moveit2
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    # ros-$ROS_DISTRO-rqt_joint_trajectory_controller \
    ros-$ROS_DISTRO-moveit-*


#####################################################################
# Install Doosan Robotics
#
# https://www.doosanrobotics.com/en
# https://github.com/DoosanRobotics/doosan-robot2
# Gazebo: ros2 launch dsr_gazebo2 dsr_gazebo.launch.py

FROM stage-neobotix AS stage-doosan

ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV DOOSAN_WS=$HOME/doosan_ws

RUN apt-get update && apt-get install -y \
    libpoco-dev libyaml-cpp-dev wget \
    ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
    ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
    ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
    dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
    ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control

RUN apt-get update && apt-get install -y \
    libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ros-gz

RUN mkdir -p $DOOSAN_WS/src && \
    cd $DOOSAN_WS/src && \
    git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git && \
    cd $DOOSAN_WS && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y && \
    colcon build && colcon build && colcon build


#####################################################################
# Finalization

FROM stage-doosan AS stage-finalization

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

# Place this file in the root of your locally mapped folder (e.g., course_dir)
# This file is sourced automatically by .bashrc
# It allows further ROS customization if needed without rebuilding the image

### Turtlebot3
# https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel
# https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup
# https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
#
# export ROS_DOMAIN_ID=30 #TURTLEBOT3
# export TURTLEBOT3_MODEL=burger # for simulation
# export TURTLEBOT3_MODEL=waffle # for simulation


### Husky
# See the following link for some examples of Husky customization
# environment variables
# http://wiki.ros.org/husky_bringup/Tutorials/Customize%20Husky%20Configuration
#
# Enable the SICK LMS1XX LIDAR on Husky
# export HUSKY_LMS1XX_ENABLED=true


### Setting up colcon_cd
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/

### Aliases
alias ros2pkgcreate="ros2 pkg create --build-type ament_python --license Apache-2.0"

# enable color support of ls and also add handy aliases
# https://askubuntu.com/questions/17299/what-do-the-different-colors-mean-in-ls
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    alias dir='dir --color=auto'
    alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi


### Update path
if [ -d "$HOME/.local/bin" ] ; then
  PATH="$PATH:$HOME/.local/bin"
fi

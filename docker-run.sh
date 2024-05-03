#!/bin/sh

docker run -it --rm --privileged --name docker-ros2-humble-elg5228  \
       -p 6080:80 \
       --security-opt seccomp=unconfined \
       --shm-size=512m \
       --volume ~/OneDrive-uOttawa/Docker-ELG5228-ROS2/course_dir:/home/ubuntu/ros2_ws/src/course_dir \
       realjsk/docker-ros2-humble-elg5228:20240503   \
       bash

# Browse http://127.0.0.1:6080/



# docker run -it --rm --privileged --name docker-ros2-elg5228 \
#        -p 6080:80 \
#        --security-opt seccomp=unconfined --shm-size=512m \
#        realjsk/docker-ros2-elg5228:test \
#        bash

# docker run -it --rm --privileged --name docker-ros2-elg5228 \
#        -p 5901:5901 \
#        -p 6901:6901 \
#        --volume ~/OneDrive-uOttawa/Docker-ELG5228-ROS2/course_dir:/home/ros/catkin_ws/src/course_dir \
#        realjsk/docker-ros2-elg5228:test \
#        bash

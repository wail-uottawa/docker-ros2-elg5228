#!/bin/sh

docker run -it --rm --privileged --name docker-ros2-humble-elg5228 \
       -p 6080:80 \
       --security-opt seccomp=unconfined \
       --shm-size=512m \
       --volume ~/course_dir:/home/ubuntu/ros2_ws/src/course_dir \
       realjsk/docker-ros2-humble-elg5228:20250616 \
       bash

# Browse http://127.0.0.1:6080/

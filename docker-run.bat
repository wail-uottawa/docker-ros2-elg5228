docker run -it --rm --privileged --name docker-ros2-humble-elg5228 -p 6080:80  --security-opt seccomp=unconfined  --shm-size=512m  --volume /C/OneDrive-uOttawa/Docker-ELG5228-ROS2/course_dir:/home/ubuntu/ros2_ws/src/course_dir:rw    realjsk/docker-ros2-humble-elg5228:20240503   bash
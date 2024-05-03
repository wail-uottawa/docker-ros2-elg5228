#!/bin/bash

# Independent squashing tool
# https://github.com/goldmann/docker-squash

# docker build --target stage-husky  -t realjsk/docker-ros2-elg5228:test    .
# docker build --squash --target stage-finalization -t realjsk/docker-ros-elg5228:20210908 --no-cache  .
# docker build --squash -t realjsk/docker-ros-elg5228:20210908  .


docker buildx build --platform=linux/amd64 --progress=plain  -t realjsk/docker-ros2-humble-elg5228:20240503  --no-cache  .

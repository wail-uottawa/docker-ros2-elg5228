#!/bin/bash

# Independent squashing tool
# https://github.com/goldmann/docker-squash

# docker build --squash --target stage-finalization -t realjsk/docker-ros2-humble-elg5228:20240503 --no-cache  .

docker buildx build --platform=linux/amd64 --progress=plain  -t realjsk/docker-ros2-humble-elg5228:20240503  --no-cache  .

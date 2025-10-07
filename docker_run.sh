#!/bin/bash

docker run --rm -it \
  --device=/dev/video1:/dev/video0 \
  --device=/dev/video11:/dev/video1 \
  --device=/dev/video13:/dev/video2 \
  webcam_ros2 \
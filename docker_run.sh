#!/bin/bash

xhost +local:root

docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/video1:/dev/video0 \
  --device=/dev/video6:/dev/video1 \
  --device=/dev/video8:/dev/video2 \
  webcam_ros2 \
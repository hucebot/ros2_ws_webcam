#!/bin/bash

xhost +local:docker

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env XAUTHORITY=$XAUTHORITY \
  --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --env QT_X11_NO_MITSHM=1 \
  --env ROS_DOMAIN_ID=39 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --net host \
  --ipc host \
  --pid host \
  --device /dev/video0:/dev/video0 \
  --device /dev/video8:/dev/video1 \
  --device /dev/video10:/dev/video2 \
  -v /media/panda-ai/Data/rosbags:/rosbags \
  webcam_ros2 \

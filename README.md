# Docker

It is a tool to package and run software in isolated environments.

- **Image**: a snapshot of a filesystem + instructions for running. You build images with docker build.
- **Container**: a running instance of an image. You create/run containers from an image.

## 1. Dockerfile

```bash
# Use official ROS2 Humble base image
FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
	ros-humble-image-transport \
    ros-humble-compressed-image-transport \
 && rm -rf /var/lib/apt/lists/*

# Copy your ROS2 workspace into container
COPY ros2_ws_webcam /ros2_ws_webcam

# Build the workspace
WORKDIR /ros2_ws_webcam
RUN . /opt/ros/humble/setup.sh && colcon build

# Source ROS2 and workspace
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws_webcam/install/setup.bash" >> ~/.bashrc
```

## 2. Build docker image

From folder where you have Dockerfile:

```bash
docker build -t webcam_ros2 .
```

- -t: webcam_ros2: gives a name to the image (you can choose whatever you want)
- .:means the Dockerfile is in the current directory

This step copies your workspace ros2_ws_webcam into the container, installs dependencies, and builds the workspace with colcon build.

### 2.1 Bind mount of your local workspace

If the code is copied inside the image (COPY ros2_ws_webcam /ros2_ws_webcam), every time you change a line you need to rebuild the image.

Solution: remove COPY from the Dockerfile and just mount the local folder with the docker run command:

```bash
docker run --rm -it \
  -v ~/Documents/Coding/ros2_ws_webcam:/ros2_ws_webcam \
  webcam_ros2 \
```

Inside the container you will see the same folder from your host (/ros2_ws_webcam).

You can edit from your PC with VSCode, then inside the container just run:

```bash
cd /ros2_ws_webcam
colcon build
source install/setup.bash
ros2 run webcam_pkg webcam_node
```

Now you do not have to rebuild image every time you modify the code.

### 2.2 Images info & delete

To see images:

```bash
docker image ls
```

To see containers:

```bash
docker ps -a
```

To delete images:

```bash
docker rmi <image_id_or_name>
```

To delete containers:

```bash
docker rm -f <container_id_or_name>  # forza la rimozione fermando il container
docker container prune # delete all unused containers
```

## 3. Docker run

Start the container launching the following command (from any folder):

```bash
docker run --rm -it \
    --device=/dev/video9 \
    --device=/dev/video10 \
    webcam_ros2
```

- --rm: deletes the container when it stops
- -it: interactive mode (gives you a shell)
- webcam_ros2: name of the image created in step 1

### 3.1 Exit from a running container

To exit running container, just digit:

```bash
exit
```

### 3.2 Docker run via script bash 

Create script bash file:

```bash
touch docker_run.sh
```

Inside it you should put the docker run command with all the required settings (device, network host, etc).

Make the file executable:

```bash
chmod +x docker_run.sh
```

After that you can launch it with:

```bash
./docker_run.sh
```

## 4. Docker compose

Assume you want to run both a webcam node and a Realsense node in separate containers (i.e., each coming from a different Docker image/Dockerfile: one containing the webcam code and the other containing the Realsense code).

You can launch them either with two separate docker run commands or with Docker Compose.

However, in both cases, the topics published by each node are automatically visible to the other container only when run from the same laptop. Otherwise, if you want the nodes to communicate, you need additional configuration, such as:

- Using --network host mode
- Configuring the same ROS_DOMAIN_ID

This ensures that the ROS2 nodes across containers can see each other and exchange topics.

```bash
version: "3.9"

services:
  webcam:
    image: webcam_ros2
    devices:
      - "/dev/video0:/dev/video0"
      - "/dev/video1:/dev/video1"
    command: >
      ros2 launch webcam_pkg my_launch_file.launch.py
      video_device1:=/dev/video0
      video_device2:=/dev/video1

  realsense:
    image: realsense_ros2
    devices:
      - "/dev/video2:/dev/video2"
    command: >
      ros2 launch realsense2_camera rs_launch_file.launch.py
```

### 4.1 Compose usage

1) Separate repositories/images:

    One repo (or pre-built image) for the webcam node, containing its Dockerfile.

    Another repo (or pre-built image) for the Realsense node, with its Dockerfile.

    The other person either clones the repos and builds the images locally using docker build, or you provide pre-built images on Docker Hub/registry.

2) docker-compose folder:

    In a separate, empty folder, you put just the docker-compose.yml that references those images.

    The docker-compose.yml defines both services (webcam and Realsense), mounts devices, sets network mode, etc.

3) Running the setup:

    From that folder, they run:

    ```bash
    docker compose up
    ```

    Compose will start two containers (or more if you define more nodes) based on the images.

4) To stop (and delete) containers activated through compose:

    ```bash
    docker compose down
    ```

### 4.2 Compose with multiple shells

To launch your nodes and keep them running in the background, you can use compose in detached mode:

```bash
docker compose up -d
```

Then you can open a shell in the running container whenever you want to digit new commands (rostopic list, roslaunch, ...):

```bash
docker compose exec webcam bash
```

## 5. Cameras

### 5.1 Realsense

Installation guide for realsense:

https://github.com/IntelRealSense/realsense-ros

To check whether realsense is connected:

```bash
lsusb | grep RealSense
```

Once inside the container, you can start your ROS2 node by launching realsense node:

```bash
ros2 launch realsense2_camera rs_launch.py
```

```bash
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=640,480,30 \
    depth_module.depth_profile:=640,480,30 \
    align_depth.enable:=true
```

If you disable depth, you can reach up to 40Hz:

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=false \
    rgb_camera.color_profile:=640,480,60
```

Compute pointcloud:

```bash
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=640,480,60 \
    depth_module.depth_profile:=640,480,60 \
    align_depth.enable:=true \
    pointcloud.enable:=true
 ```

**Note**: To visualize the pointcloud in RViz, select the frame = camera_link.

Important topics:

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw/compressed
```

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/image_raw/compressed
```

To visualize:

```bash
ros2 run rqt_image_view rqt_image_view
```

#### 5.1.1 Realsense issues

ros2 topic echo /camera/camera/color/image_raw --once
 
--> vedo che ripubblica jpeg , come cambio in png??

### 5.2 Webcam

List video devices:

```bash
v4l2-ctl --list-devices
```

Get info about video devices:

```bash
v4l2-ctl --list-formats-ext -d /dev/video9
```

Get real stream frequency:

```bash
v4l2-ctl --device=/dev/video9 --stream-mmap --stream-count=100
```

Once inside the container, you can start your ROS2 node by launching multiple webcam publishers:

```bash
ros2 launch launcher_pkg launch_file.py \
    video_device1:=/dev/video0 \
    video_device2:=/dev/video1 \
    video_device3:=/dev/video2 \
    image_width:=640 \
    image_height:=480 \
    framerate:=15 \
    transport_on:=true
```

Since the webcame node publishes compressed PNG images, I need transport node to convert them in Image type to display on RVIz. This is done by enabling ''transport_on:=true'' in the roslaunch command. 

Otherwise, from terminal:

```bash
ros2 run image_transport republish compressed raw \
    --ros-args -r in/compressed:=/webcam1/image_raw \
               -r out:=/webcam1/image_raw/compressed
```
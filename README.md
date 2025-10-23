# Webcam docker

This docker file can be used to setup automatically webcams and make them publish topics on ROS2 Humble.

## 1. Build docker

Digit the following command (in the folder where **Dockerfile** is located) to create the docker image:

```bash
docker build -t webcam_ros2 .
```

## 2. Access container

From the same folder, digit the following command:

```bash
./docker_run.sh
```

## 3. Start webcams

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

**Note**: This command must be modified based on the number of webcams and based on the device port numbers (```v4l2-ctl --list-devices```).

Since the webcame node publishes compressed PNG images, I need transport node to convert them in Image type to display on RVIz. This is done by enabling ''transport_on:=true'' in the roslaunch command. 

Otherwise, from terminal:

```bash
ros2 run image_transport republish compressed raw \
    --ros-args -r in/compressed:=/webcam1/image_raw \
               -r out:=/webcam1/image_raw/compressed
```

To visualize:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Docker deep dive

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
docker rm -f <container_id_or_name>  # force cancellation
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
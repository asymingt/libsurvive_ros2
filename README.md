# libsurvive_ros2

This is a lightweight ROS2 wrapper around the [libsurvive](https://github.com/cntools/libsurvive) project, which provides a set fo drivers for 6DoF rigid body tracking using SteamVR 1.0 and 2.0 hardware. This hardware is particularly useful to robotics projects, because it provides a cost effective method of obtaining ground truth with a positional accuracy typically in the sub-centimeter, sub-degree range.

The driver in this repo is largely based on the ROS1 driver available from [libsurvive](https://github.com/cntools/libsurvive/tree/master/tools/ros_publisher). It has been migrated to ROS2, and refactored slightly -- we use a thread to manage the blocking interaction with libsurvive, so that it doesn't lock the ROS2 callback queue and prevent messages from propagating correctly.

When you build this code `cmake` will checkout and build the latest stable release of `libsurvive` and link against this library for you. This is to avoid having to discover it through pkg-config, and ensures that you are using a version that is known to work with the ROS2 driver.

# Installation instructions

This has only been tested on Ubuntu 22.04 and ROS Humble, although its fairly likely to work correctly with other distributions too. Pull requests are welcome if it does not!

Before you do anything, you will need to have these installed on your machine.

```
sudo curl -fsSL https://raw.githubusercontent.com/cntools/libsurvive/master/useful_files/81-vive.rules \
    -o /etc/udev/rules.d/81-vive.rules
sudo udevadm control --reload-rules && udevadm trigger
```

You can now choose to build the driver natively or in a container. The benefit of launching it within a container is that it won't mess with any pre-existing ROS installation on your machine. However, you will need docker-ce and the compose plugin for things to work well. Instructions are included below.

# Native installation (easiest)

You'll need ROS2 installed: https://docs.ros.org/en/humble/Installation.html

You'll also need to follow the instructions here: 

```sh
sudo apt-get install build-essential \
    zlib1g-dev \
    libx11-dev \
    libusb-1.0-0-dev \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    libatlas-base-dev cmake
```

# Containerized installation (easiest)

Install docker: https://docs.docker.com/engine/install/ubuntu/

```sh
docker build --tag libsurvive_ros2:master .
docker compose up
```

# Visualization with Foxglove
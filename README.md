# libsurvive_ros2

This is a lightweight ROS2 wrapper around the [libsurvive](https://github.com/cntools/libsurvive) project, which provides a set fo drivers for 6DoF rigid body tracking using SteamVR 1.0 and 2.0 hardware. Such hardware is particularly useful to robotics projects, because it provides a relatively cheap method of obtaining ground truth with a positional accuracy in the or millimeters and a rotational accuracy in the order of degrees.

# Installation instructions

You'll need ROS2 installed: https://docs.ros.org/en/humble/Installation.html

You'll also need to follow the instructions here: 

```sh
sudo curl -fsSL https://raw.githubusercontent.com/cntools/libsurvive/master/useful_files/81-vive.rules \
    -o /etc/udev/rules.d/81-vive.rules
sudo udevadm control --reload-rules && udevadm trigger
sudo apt-get install build-essential \
    zlib1g-dev \
    libx11-dev \
    libusb-1.0-0-dev \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    libatlas-base-dev cmake
```

# How it works
# MIT License
#
# Copyright (c) 2023 Andrew Symington
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

ARG ROS_DISTRO=humble
ARG ARCH=amd64

# Default image, user and root directory
FROM ${ARCH}/ros:${ROS_DISTRO}-ros-base
SHELL ["/bin/bash", "-c"]

# Install baseline tools
RUN apt-get update && apt-get install -y --no-install-recommends                        \
        build-essential                                                                 \
        ccache                                                                          \
        cmake                                                                           \
        freeglut3-dev                                                                   \
        gdb                                                                             \
        libboost-all-dev                                                                \
        libatlas-base-dev                                                               \
        liblapacke-dev                                                                  \
        libopenblas-dev                                                                 \
        libtbb-dev                                                                      \
        libpcap-dev                                                                     \
        libusb-1.0-0-dev                                                                \
        libx11-dev                                                                      \
        libyaml-cpp-dev                                                                 \
        python3-pip                                                                     \
        python3-scipy                                                                   \
        ros-${ROS_DISTRO}-foxglove-bridge                                               \
        ros-${ROS_DISTRO}-gtsam                                                         \
        ros-${ROS_DISTRO}-launch-pytest                                                 \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap                                          \
        sudo                                                                            \
        valgrind                                                                        \
        xterm                                                                           \
        zlib1g-dev                                                                      \
    && pip3 install gtsam                                                               \
    && sudo rm -rf /var/lib/apt/lists/*

# Add an 'ubuntu' user with dialout/plugdev access and can use sudo passwordless.
RUN useradd -ms /bin/bash ubuntu && echo "ubuntu:ubuntu" | chpasswd
RUN usermod -aG sudo,dialout,plugdev ubuntu
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the non-root user.
USER ubuntu

# Copy the source code into our test workspace.
RUN mkdir -p /home/ubuntu/ros2_ws/src/libsurvive_ros2
COPY --chown=ubuntu:ubuntu . /home/ubuntu/ros2_ws/src/libsurvive_ros2

# Compile the code. Normally you'd also add these, but they are not cached by docker,
# which makes compilation long and tedious.
#
#   rosdep update                                                                       \
#   rosdep install --from-paths src --ignore-src -r -y                                  \
#   source /opt/ros/$ROS_DISTRO/setup.bash                            
#
RUN sudo apt-get update                                                                 \
    && source /opt/ros/$ROS_DISTRO/setup.bash                                           \
    && cd /home/ubuntu/ros2_ws                                                          \
    && colcon build --symlink-install --event-handlers console_direct+                  \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo                                  \
    && sudo rm -rf /var/lib/apt/lists/*

# Initialization
RUN echo -e "#!/bin/bash \n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source /home/ubuntu/ros2_ws/install/setup.bash \n\
exec \$@" > /home/ubuntu/ros2_ws/entrypoint.sh
RUN chmod 755 /home/ubuntu/ros2_ws/entrypoint.sh
WORKDIR /home/ubuntu/ros2_ws


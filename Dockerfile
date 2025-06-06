# MIT License
#
# Copyright (c) 2025 Andrew Symington
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

# Default user and group information
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# Default shell to use in the RUN commands below.
SHELL ["/bin/bash", "-c"]

# Grab the latest ROS2 key to avoid an outdated key from affecting the build.
# See: https://github.com/osrf/docker_images/issues/807.
RUN rm -rf                                                                      \
    /etc/apt/sources.list.d/ros2.sources                                        \
    /etc/apt/sources.list.d/ros2-snapshots.list                                 \
 && apt-get --allow-unauthenticated --allow-insecure-repositories update        \
 && apt-get install -y --no-install-recommends curl                             \
 && sudo rm -rf /var/lib/apt/lists/*                                            \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key    \
    -o /usr/share/keyrings/ros2-snapshots-archive-keyring.gpg                   \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-snapshots-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install baseline tools
RUN apt-get update                                                              \
 && apt-get install -y --no-install-recommends                                  \
        build-essential                                                         \
        cmake                                                                   \
        freeglut3-dev                                                           \
        gdb                                                                     \
        liblapacke-dev                                                          \
        libopenblas-dev                                                         \
        libpcap-dev                                                             \
        libusb-1.0-0-dev                                                        \
        libx11-dev                                                              \
        sudo                                                                    \
        valgrind                                                                \
        zlib1g-dev                                                              \
    && sudo rm -rf /var/lib/apt/lists/*

# Replace any existing 'ubuntu' user with a 'ros' user with specific privileges.
RUN groupdel ubuntu || true                                                     \
 && userdel -r ubuntu || true                                                   \
 && groupadd --gid $USER_GID $USERNAME                                          \
 && useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash -m $USERNAME      \
 && usermod -aG sudo,dialout,plugdev $USERNAME                                  \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the non-root user.
USER ros

# Copy the source code into our test workspace.
RUN mkdir -p /home/ros/.config /home/ros/ros2_ws/src/libsurvive_ros2
COPY --chown=ros:ros . /home/ros/ros2_ws/src/libsurvive_ros2

# Install baseline tools
RUN sudo apt-get update                                                         \
 && cd /home/ros/ros2_ws                                                        \
 && rosdep update                                                               \
 && rosdep install --from-paths src --ignore-src -r -y                          \
 && source /opt/ros/$ROS_DISTRO/setup.bash                                      \
 && colcon build                                                                \
 && sudo rm -rf /var/lib/apt/lists/*

# Initialization
RUN echo -e "#!/bin/bash \n\
set -e\n\
source /home/ros/ros2_ws/install/setup.bash \n\
exec \$@" > /home/ros/ros2_ws/entrypoint.sh
RUN chmod 755 /home/ros/ros2_ws/entrypoint.sh
WORKDIR /home/ros/ros2_ws
ENTRYPOINT [ "/home/ros/ros2_ws/entrypoint.sh" ]


# MIT License
#
# Copyright (c) 2022 Andrew Symington
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

# Default image, user and root directory
FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# Install baseline tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    freeglut3-dev \
    libatlas-base-dev \
    liblapacke-dev \
    libopenblas-dev \
    libusb-1.0-0-dev \
    libx11-dev \
    zlib1g-dev

# Add an 'ubuntu' user with dialout and plugdev access
RUN useradd -ms /bin/bash ubuntu && echo "ubuntu:ubuntu" | chpasswd
RUN usermod -aG sudo,dialout,plugdev ubuntu
USER ubuntu
WORKDIR /home/ubuntu

# Copy the code
RUN mkdir -p /home/ubuntu/ros2_ws/src/libsurvive_ros2
#COPY --chown=ubuntu:ubuntu . /home/ubuntu/ros2_ws/src/libsurvive_ros2
#RUN cd /home/ubuntu/ros2_ws \
# && source /opt/ros/humble/setup.bash \
# && colcon build --symlink-install --event-handlers console_direct+

# Initialization
# RUN echo -e "#!/bin/bash \n\
# set -e\n\
# source /opt/ros/rolling/setup.bash \n\
# exec \$@" > /home/libsurvive_ros2_ws/entrypoint.sh
# RUN chmod 755 /root/entrypoint.sh
# ENTRYPOINT [ "/root/entrypoint.sh" ]

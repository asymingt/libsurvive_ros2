# Copyright 2023 Andrew Symington
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Information about the origin of this data
# > stamp: precise time of capture
# > frame_id: <tracker_id>[<sensor_id>]
std_msgs/Header header

# Index of this lighthouse
uint8 index

# Operating 'channel' or 'mode' (1-16)
uint8 channel

# Number of times system has been unlocked.
uint8 unlock_count

# Acceleration reading
geometry_msgs/Vector3 accel

# Lighthouse calibration parameters
float64[2] fcalphase
float64[2] fcaltilt
float64[2] fcalcurve
float64[2] fcalgibpha
float64[2] fcalgibmag
float64[2] fcalogeephase
float64[2] fcalogeemag

# Need to work out what these are for
uint64 ootx_set
uint64 position_set

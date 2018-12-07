#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, HLP-R
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#  contributors may be used to endorse or promote products derived
#  from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Baris Akgun 

import roslib; 
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import time
from math import *

def clamp(x, limits):
  return max(min(x, limits[1]), limits[0])

class PanTilt:
  def __init__(self, pan_limits=None, tilt_limits=None, queue_size=10):
    if pan_limits is None:
      self.pan_limits  = [-pi/3.,pi/2.]
    else:
      self.pan_limits = pan_limits

    if tilt_limits is None:
      self.tilt_limits = [-pi/3., pi/3.]
    else:
      self.tilt_limits = tilt_limits

    self.pub_tilt = rospy.Publisher('/tilt_controller/command', Float64, queue_size=queue_size)
    self.pub_pan  = rospy.Publisher('/pan_controller/command', Float64, queue_size=queue_size)
    
    rospy.Subscriber("/tilt_controller/state", JointState, self.cb_tilt)
    rospy.Subscriber("/pan_controller/state", JointState, self.cb_pan)

    self.tilt_pos = 0.0
    self.pan_pos = 0.0

  def set_pan(self, pan_position):
    ros_rate = rospy.Rate(rate)
    pos = clamp(pos, self.pan_limits)
    self.pub_pan.publish(pan_position)

  def set_tilt(self, tilt_position):
    ros_rate = rospy.Rate(rate)
    pos = clamp(pos, self.tilt_limits)
    self.pub_tilt.publish(tilt_position)

  def set_pan_tilt(self, pan_position, tilt_position):
    ros_rate = rospy.Rate(rate)
    pos[0] = clamp(pos[0], self.pan_limits)
    pos[1] = clamp(pos[1], self.tilt_limits)
    self.pub_pan.publish(pan_position)
    self.pub_tilt.publish(tilt_position)

  def cb_tilt(self, js):
    self.tilt_pos = js.position[0]

  def cb_pan(self, js):
    self.pan_pos = js.position[0]

if __name__ == "__main__":
  rospy.init_node('pan_tilt_test', anonymous=True)
  pt = PanTilt()
  search_pattern = [[0.0,0.7], [0.3, 0.6],[-0.3, 0.6],[-0.3, 0.8],[0.3, 0.8]]
  for i in range(0, len(search_pattern)):
    pt.set_pan_tilt(search_pattern[i])
    time.sleep(0.5)
  pt.set_pan_tilt([0,0])


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
from dynamixel_msgs.msg import JointState
import rospy
import time
from math import *

def clamp(x, limits):
  return max(min(x, limits[1]), limits[0])

class PanTilt:
  def __init__(self):

    self.pubTilt = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
    self.pubPan  = rospy.Publisher('/pan_controller/command', Float64, queue_size=10)
    
    self.subTilt = rospy.Subscriber("/tilt_controller/state", JointState, self.cb_tilt)
    self.subPan = rospy.Subscriber("/pan_controller/state", JointState, self.cb_pan)

    self.tilt_pos = 0.0
    self.pan_pos = 0.0

    self.pan_limits  = [-pi/3.,pi/2.]
    self.tilt_limits = [-pi/3., pi/3.]

  def set_pan(self, pos, repetitions = 10, rate = 10):
    ros_rate = rospy.Rate(rate)
    pos = clamp(pos, self.pan_limits)
    for i in range(0,repetitions):
      self.pubPan.publish(pos)
      ros_rate.sleep()

  def set_tilt(self, pos, repetitions = 10, rate = 10):
    ros_rate = rospy.Rate(rate)
    pos = clamp(pos, self.tilt_limits)
    for i in range(0,repetitions):
      self.pubTilt.publish(pos)
      ros_rate.sleep()

  def set_pantilt(self, pos, repetitions = 10, rate = 10):
    ros_rate = rospy.Rate(rate)
    pos[0] = clamp(pos[0], self.pan_limits)
    pos[1] = clamp(pos[1], self.tilt_limits)
    for i in range(0,repetitions):
      self.pubPan.publish(pos[0])
      self.pubTilt.publish(pos[1])
      ros_rate.sleep()

  def cb_tilt(self, js):
    self.tilt_pos = js.current_pos

  def cb_pan(self, js):
    self.pan_pos = js.current_pos

if __name__ == "__main__":
  rospy.init_node('pantilt_test')
  pt = PanTilt()
  searchPattern = [[0.0,0.7], [0.3, 0.6],[-0.3, 0.6],[-0.3, 0.8],[0.3, 0.8]]
  for i in range(0, len(searchPattern)):
    pt.set_pantilt(searchPattern[i])
    time.sleep(0.5)
  pt.set_pantilt([0,0])


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
import os
from math import *

def clamp(x, limits):
  return max(min(x, limits[1]), limits[0])

class PanTilt:
  #limits in form min, max, zero point
  def __init__(self, pan_limits=None, tilt_limits=None, queue_size=10):
    self.robot = rospy.get_param("/ROBOT_NAME", "default")
    
    if pan_limits is None and self.robot=="poli2":
      self.pan_limits  = [-3.14,3.14, 0]
    elif pan_limits is None:
      self.pan_limits  = [-pi/3.,pi/2., 0]
    else:
      self.pan_limits = pan_limits

    if tilt_limits is None and self.robot=="poli2":
      self.tilt_limits  = [0.0,0.9,0.65] #[0.0,1.2,0.65]<-use once wires go through neck
      self.tilt_conv = lambda x: -x
    elif tilt_limits is None:
      self.tilt_limits = [-pi/3., pi/3., 0]
      self.tilt_conv = lambda x: x
    else:
      self.tilt_limits = tilt_limits

    #pan/tilt position in unknown state to start
    self.tilt_pos = None
    self.pan_pos = None

    rospy.Subscriber("/tilt_controller/state", JointState, self.cb_tilt)
    rospy.Subscriber("/pan_controller/state", JointState, self.cb_pan)

    if self.robot=="poli2":
      self.pub_tilt = rospy.Publisher('/tilt_motor/position_controller/command', Float64, queue_size=queue_size)
      self.pub_pan  = rospy.Publisher('/pan_motor/position_controller/command', Float64, queue_size=queue_size)
      rospy.Subscriber("/tilt_motor/joint_states", JointState, self.cb_tilt)
      rospy.Subscriber("/pan_motor/joint_state", JointState, self.cb_pan)
    else:
      self.pub_tilt = rospy.Publisher('/tilt_controller/command', Float64, queue_size=queue_size)
      self.pub_pan  = rospy.Publisher('/pan_controller/command', Float64, queue_size=queue_size)
    rospy.sleep(0.5)


    while self.pan_pos is None or self.tilt_pos is None:
      rospy.sleep(0.1)
      rospy.logwarn_throttle(1,"Waiting for pan/tilt position publishers...")
    
    
  def set_pan(self, pan_position, wait=False):
    pan_position += self.pan_limits[2]
    
    pos = clamp(pos, self.pan_limits)
    self.pub_pan.publish(pan_position)
    if wait:
      self.wait()

  def set_tilt(self, tilt_position, wait=False):
    pos = clamp(tilt_position + self.tilt_limits[2], self.tilt_limits)
    if self.robot=="poli2":
      tilt = -1*tilt
    self.pub_tilt.publish(self.tilt_conv(tilt_position))
    if wait:
      self.wait()

  def set_pan_tilt(self, pan_position, tilt_position, wait=False):
    print "requested pan/tilt:", pan_position, tilt_position 
    pan = clamp(pan_position + self.pan_limits[2], self.pan_limits)
    tilt = clamp(tilt_position + self.tilt_limits[2], self.tilt_limits)
    if self.robot=="poli2":
      tilt = -1*tilt
    print "setting pan/tilt:", pan, tilt
    self.pub_pan.publish(pan)
    self.pub_tilt.publish(tilt)
    if wait:
      self.wait()

  #could be up to 0.5s delayed from actual end of movement
  def wait(self):
    changed = True
    self.prev_pan = self.pan_pos
    self.prev_tilt = self.tilt_pos
    while(changed):
      rospy.sleep(0.5)
      d_pan = abs(self.pan_pos-self.prev_pan)
      d_tilt = abs(self.tilt_pos-self.prev_tilt)
      if d_pan<0.01 and d_tilt<0.01:
        changed = False
      self.prev_pan = self.pan_pos
      self.prev_tilt = self.tilt_pos

  def cb_tilt(self, js):
    self.tilt_pos = js.position[0]

  def cb_pan(self, js):
    self.pan_pos = js.position[0]

if __name__ == "__main__":
  rospy.init_node('pan_tilt_test', anonymous=True)
  pt = PanTilt()
  search_pattern = [[0.0,-0.8], [0.3, 0.3],[-0.3, 0.2],[-0.3, -0.8],[0.3, 0.1]]
  for i in range(0, len(search_pattern)):
    pt.set_pan_tilt(search_pattern[i][0], search_pattern[i][1], wait=True)
    print("Done")
    rospy.sleep(2.0)
  pt.set_pan_tilt(0,0, wait=True)


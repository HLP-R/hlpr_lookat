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
# An example of hot to follow the end effector without using the lookout service
#
# Authors: Baris Akgun 

import roslib; 
from geometry_msgs.msg import *
import rospy
import tf2_ros
import time
from hlpr_lookat import *
from hlpr_lookat.look_at_kinematics import *
from hlpr_lookat.pantilt import *

class FollowEef:
  def __init__(self, root = 'base_link', ee_frame = 'right_ee_link', pt_base = 'pan_base_link', kinect_frame = 'kinect_ir_optical_frame'):
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    self.transform_root = root
    self.ee_frame = ee_frame
    self.pt_base = pt_base
    self.kinect_frame = kinect_frame
    
    self.latest_base_ee_transform = None
    self.latest_pt_ee_transform = None

    self.pt = PanTilt()
    self.head = LookAtKin()
    self.head.calc_ik_im_trans = True

  def updateTransform(self):
    try:
      self.latest_base_ee_transform = self.tf_buffer.lookup_transform(self.transform_root, self.ee_frame, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print 'Cannot lookup transform from base to end effector:'
      return False

    try:
      #self.latest_pt_ee_transform = self.tf_buffer.lookup_transform(self.kinect_frame, self.ee_frame, rospy.Time(0))
      self.latest_pt_ee_transform = self.tf_buffer.lookup_transform(self.pt_base, self.ee_frame, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print 'Cannot lookup transform from pan_tilt base to end effector:'
      return False

    return True


  def lookAtHand(self):
    if(self.updateTransform()):
      pos = [self.latest_pt_ee_transform.transform.translation.x,self.latest_pt_ee_transform.transform.translation.y,self.latest_pt_ee_transform.transform.translation.z]
      #theta = self.head.headIK(self.head.baseToObject(pos, [self.pt.pan_pos, self.pt.tilt_pos]))
      theta = self.head.headIK(pos)
      self.pt.set_pantilt(theta)
      return theta
    return None

if __name__ == '__main__':
  rospy.init_node('follow_eef_test')
  feef = FollowEef()
  time.sleep(1)
  rate =rospy.Rate(50)
  while not rospy.is_shutdown():
    theta = feef.lookAtHand()
  

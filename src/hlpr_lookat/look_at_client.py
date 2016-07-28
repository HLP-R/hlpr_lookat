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
# An example of how to use the lookout service
#
# Authors: Baris Akgun 

import sys
import rospy
import tf2_ros
import time

from hlpr_lookat.srv import LookAt, LookAtT, LookAtTS
from geometry_msgs.msg import Vector3


def look_at_client(pos, service_name = 'lookat_vec3', msg_type = LookAt):
  rospy.wait_for_service(service_name)
  try:
    lookat = rospy.ServiceProxy(service_name, msg_type)
    resp = lookat(pos)
    return resp.success
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def get_ee_frame():
  tf_buffer = tf2_ros.Buffer()
  tf_listener = tf2_ros.TransformListener(tf_buffer)
  time.sleep(1)

  transform_root = 'base_link'
  pt_base        = 'pan_base_link'
  ee_frame       = 'right_ee_link'

  try:
    latest_base_ee_transform = tf_buffer.lookup_transform(transform_root, ee_frame, rospy.Time(0))
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print 'Cannot lookup transform from base to end effector:'
    return None

  try:
    #latest_pt_ee_transform = tf_buffer.lookup_transform(kinect_frame, ee_frame, rospy.Time(0))
    latest_pt_ee_transform = tf_buffer.lookup_transform(pt_base, ee_frame, rospy.Time(0))
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print 'Cannot lookup transform from pan_tilt base to end effector:'
    return None

  return (latest_base_ee_transform, latest_pt_ee_transform)

#this is 
if __name__ == "__main__":
  rospy.init_node('lookat_client')
  a = get_ee_frame()
  b_ee = a[0]
  pt_ee = a[1]

  #look_at_client(b_ee.transform.translation)
  #look_at_client(b_ee.transform, 'lookat_tr', LookAtT)
  look_at_client(pt_ee, 'lookat_s_tr', LookAtTS)


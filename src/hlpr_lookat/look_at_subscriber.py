#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, HLP-R
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

import time

import rospy
import tf2_ros
from std_msgs.msg import Float32

from hlpr_lookat.pan_tilt import PanTilt

class LookAtSubscriber:
  def __init__(self, name='lookat', spin=True):
    self.pt = PanTilt()
    self.name = name
    self.pan_listener = rospy.Subscriber(self.name + "/pan", Float32, self.cb_pan)
    self.tilt_listener = rospy.Subscriber(self.name + "/tilt", Float32, self.cb_tilt)
  
    if spin:
      rospy.spin()

  def cb_pan(self, pan):
      self.pt.set_pan(pan.data)

  def cb_tilt(self, tilt):
      self.pt.set_tilt(tilt.data)

if __name__ == "__main__":
  rospy.init_node('look_at_subscriber')
  loook_at = LookAtSubscriber()
  

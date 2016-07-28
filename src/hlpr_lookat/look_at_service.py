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

import rospy
import tf2_ros
import time
import PyKDL

from geometry_msgs.msg import Vector3, Transform
from manipulation.srv import LookAt, LookAtResponse, LookAtT, LookAtTResponse, LookAtTS, LookAtTSResponse

from hlpr_lookat.look_at_kinematics import LookAtKin
from hlpr_lookat.pantilt import PanTilt

def transform_to_kdl(t):
  return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.rotation.x, t.rotation.y,
                                               t.rotation.z, t.rotation.w),
                     PyKDL.Vector(t.translation.x,
                                  t.translation.y,
                                  t.translation.z))

def transform_vec3(in_vec3, transform_frame):
  p = transform_to_kdl(transform_frame) * PyKDL.Vector(in_vec3.x, in_vec3.y, in_vec3.z)
  out_vec3 = Vector3()
  out_vec3.x = p[0]
  out_vec3.y = p[1]
  out_vec3.z = p[2]
  return out_vec3

class LookAtService:
  def __init__(self, name = 'lookat', init_service = False, base_to_pan_transform = None):
    self.pt = PanTilt()
    self.head = LookAtKin()
    self.base_name = name

    self.tfBuffer = None 
    self.listener = None 

    self.base = 'base_link'
    self.lookat_root = 'pan_base_link'

    if base_to_pan_transform is None:
      self._def_base2pantilt()
    else:
      self.base2pantilt = base_to_pan_transform


    if init_service:
      self.init_service()

  #to be used in converting from base_link to pantilt base
  def _def_base2pantilt(self):
    self.base2pantilt = Transform()
    self.base2pantilt.translation.x = -0.395
    self.base2pantilt.translation.y =  0.0
    self.base2pantilt.translation.z = -1.505

    self.base2pantilt.rotation.x = 0.0
    self.base2pantilt.rotation.y = 0.0
    self.base2pantilt.rotation.z = 0.0
    self.base2pantilt.rotation.w = 1.0
    
  def lookat(self,in_vec3):
    des_pos = transform_vec3(in_vec3, self.base2pantilt)
    theta = self.head.headIK([des_pos.x, des_pos.y, des_pos.z])
    print theta
    if theta[0] is None:
      return False
    else:
      self.pt.set_pantilt(theta)
      return True

  def handle_req_v3(self,req):
    resp = LookAtResponse()
    resp.success = self.lookat(req.desired_position)
    return resp

  def handle_req_tr(self, req):
    resp = LookAtTResponse()
    resp.success = self.lookat(req.desired_tr.translation)
    return resp

  def handle_req_s_tr(self,req):
    resp = LookAtTSResponse()
    try:
      trans = self.tfBuffer.lookup_transform(self.base, req.desired_s_tr.child_frame_id, rospy.Time(0))
      resp.success = self.lookat(trans.transform.translation)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print 'Cannot lookupTransform'
      resp.success = False

    return resp

  def init_service(self, spin = True):
    rospy.init_node(self.base_name + '_server')

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    #waiting 1 second to have these guys initialized
    time.sleep(1)

    if not isinstance(self.base2pantilt, Transform):
      print 'What are you doing?'
      try:
        trans = self.tfBuffer.lookup_transform(self.lookat_root, self.base, rospy.Time(0))
        self.base2pantilt = trans.transform
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'Cannot lookupTransform, going with default'
        self._def_base2pantilt()

    s_v3 = rospy.Service(self.base_name + '_vec3', LookAt, self.handle_req_v3)
    s_tr  = rospy.Service(self.base_name + '_tr',   LookAtT, self.handle_req_tr)
    s_s_tt = rospy.Service(self.base_name + '_s_tr', LookAtTS, self.handle_req_s_tr)

    if spin:
      rospy.spin()

if __name__ == "__main__":
  loook_at = LookAtService(init_service = True)
  

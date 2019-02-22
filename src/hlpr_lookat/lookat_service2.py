#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math

import rospy
import tf
from hlpr_lookat.pan_tilt import PanTilt
from hlpr_lookat.srv import LookAt,  LookAtRequest, LookAtResponse, LookAtT, LookAtTResponse, LookAtTS, LookAtTSResponse
from hlpr_lookat.pan_tilt import PanTilt
from geometry_msgs.msg import PoseStamped, Point
import geometry_msgs

class LookatServer:
    def __init__(self):
        self.base_name = "hlpr_lookat"
        rospy.init_node(self.base_name + '_server')
        self.pt = PanTilt(queue_size=1) #this will handle the joint limits

        self.default_frame = "base_link"
        self.head_frame = "head_link"
        
        self.listener = tf.TransformListener()
        rospy.sleep(1.0)
        
        #s_v3 = rospy.Service(self.base_name + '_vec3', LookAt, self.handle_req_v3)
        #s_tr  = rospy.Service(self.base_name + '_tr', LookAtT, self.handle_req_tr)
        #s_s_tt = rospy.Service(self.base_name + '_s_tr', LookAtTS, self.handle_req_s_tr)

    
    def handle_req_v3(self, req):
        resp = LookAtResponse()
        resp.success = self.lookat(req.desired_position)
        return resp

    def handle_req_tr(self, req):
        resp = LookAtTResponse()
        resp.success = self.lookat(req.desired_tr.translation)
        return resp

    def handle_req_s_tr(self, req):
        resp = LookAtTSResponse()
        
        resp.success = self.lookat(req.desired_s_tr.transform.translation,
                                   req.desired_s_tr.transform.rotation,
                                   req.desired_s_tr.child_frame_id)
        return resp

    # need a head link w/ z up & x out
    def pos2theta(self, pos):
        tilt = math.atan2(pos.z,pos.x)
        pan = math.atan2(pos.y,pos.x)
        return (pan, tilt)

    def lookat(self, pose, orientation = None, pos_frame = None):
        if orientation is None:
            orientation = geometry_msgs.msg.Quaternion()
            orientation.x = 0.0
            orientation.y = 0.0
            orientation.z = 0.0
            orientation.w = 1.0

        if pos_frame is None:
            pos_frame = self.default_frame
        
        try:
            # Get position from the request
            pos = PoseStamped()
            pos.pose.position = pose
            pos.header.frame_id = pos_frame
            pos.pose.orientation = orientation

            
            # Lookup the transform between frames
            trans = self.listener.transformPose(self.head_frame, pos)
            
            pan, tilt = self.pos2theta(trans.pose.position)

            print pan, tilt

            self.pt.set_pan_tilt(pan, tilt)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Cannot look up translation, not moving head.")
            return False
            
if __name__=="__main__":
    srv = LookatServer()

    req = LookAtRequest()
    req.desired_position.x = 1.0
    req.desired_position.y = 0.5
    req.desired_position.z = 1.0


    
    srv.handle_req_v3(req)

#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016-2019, HLP-R
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
# Authors: Elaine Short

import rospy
import cv2
import math
import copy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from hlpr_lookat.pan_tilt import PanTilt


class CVLookat:
    def __init__(self, display_on):
        self._bridge = CvBridge()
        self._disp = display_on
        
        self._proc_times = [0 for i in range(10)]
        self._frame_pointer = 0
        self._prev_elapsed = 0
        self._prev_time = rospy.Time.now()

        self._pt = PanTilt()

        self.robot = rospy.get_param("/ROBOT_NAME", "default")
        
        rospy.loginfo("Subscribing to image topic")
        self._sub=rospy.Subscriber("image_topic", CompressedImage, self._cb)
        
    def process_image(self, cv_img, disp_on):
        rospy.logerror("You must override the process_image function in CVLookat!")
        i,j = (None,None)
        return i,j
        
    def _cb(self,data):
        start = rospy.Time.now()
        pan,tilt = (copy.deepcopy(self._pt.pan_pos), copy.deepcopy(self._pt.tilt_pos))
        try:
            cv_image=self._bridge.compressed_imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hres = float(cv_image.shape[1])
        vres = float(cv_image.shape[0])
        
        i,j = self.process_image(cv_image, self._disp)
        
        self._frame_pointer = self._frame_pointer % len(self._proc_times)
        self._proc_times[self._frame_pointer] = self._prev_elapsed
        self._frame_pointer += 1

        time_per_frame = float(sum(self._proc_times))/float(len(self._proc_times))
        if time_per_frame != 0:
            rate = 1000000000/time_per_frame
        else:
            rate = "NA"
            
        rospy.loginfo_throttle(20,"Frame processing rate: {} fps".format(rate))
        self._prev_elapsed = (rospy.Time.now()-start).to_nsec()

        hdead = .075*hres
        vdead = .075*vres

        
        if not i is None and abs(i-hres/2)<hdead:
            i = None
            
        if not j is None and abs(j-vres/2)<vdead:
            j = None

        if self.robot=="poli2":
            hfov = math.radians(87)
            vfov = math.radians(58)
        else:
            vfov = math.radians(53.8)
            hfov = math.radians(84.1)
            
        self._pt.lookat_image_coord(i, j, hres, vres,
                                   hfov, vfov,
                                   pan, tilt)
        self._prev_time = rospy.Time.now()

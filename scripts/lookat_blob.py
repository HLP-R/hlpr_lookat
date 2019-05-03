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

import rospy
import cv2
import argparse
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

from dynamic_reconfigure.server import Server
from hlpr_lookat.cfg import ColorTrackConfig

from lookat_opencv import CVLookat

class BlobLookat(CVLookat):
    def __init__(self, display_on, invert):
        CVLookat.__init__(self, display_on)

        self._lb = np.array([0,0,0])
        self._ub = np.array([255,255,255])
        self._opening_diam = 3
        self._closing_diam = 3
        self._invert = invert
        self._reconfigure = Server(ColorTrackConfig, self._recon_cb)

    def _recon_cb(self,config, level):
        if config["opening_diam"]>=1:
            self._opening_diam = config["opening_diam"]
        else:
            self._opening_diam = 1
            config["opening_diam"]=1
        if config["closing_diam"]>=1:
            self._closing_diam = config["closing_diam"]
        else:
            self._closing_diam = 1
            config["closing_diam"]=1
            
        if config["h_min"]<=config["h_max"]:
            self._lb[0]=config["h_min"]
            self._ub[0]=config["h_max"]
        else:
            config["h_min"]=int(self._lb[0])
            config["h_max"]=int(self._ub[0])
            
        if config["s_min"]<=config["s_max"]:
            self._lb[1]=config["s_min"]
            self._ub[1]=config["s_max"]
        else:
            config["s_min"]=int(self._lb[1])
            config["s_max"]=int(self._ub[1])
            
        if config["v_min"]<=config["v_max"]:
            self._lb[2]=config["v_min"]
            self._ub[2]=config["v_max"]
        else:
            config["v_min"]=int(self._lb[2])
            config["v_max"]=int(self._ub[2])

        self._final_size_min = config["min_final_size"]
            
        return config
    
    def process_image(self, cv_image, display_on):
        hsv_lower, hsv_upper, cv_image, opening_diameter, closing_diameter = (self._lb, self._ub, cv_image, self._opening_diam, self._closing_diam)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv,hsv_lower,hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(opening_diameter,opening_diameter))
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(closing_diameter,closing_diameter))
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE, kernel)
        
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                                    cv2.CHAIN_APPROX_TC89_L1)


        image_out = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        
        blobs = sorted(contours, key=lambda el: cv2.contourArea(el))

        out_blobs = []
        for i in range(len(blobs)):
            blob = cv2.convexHull(blobs[i])
            area = cv2.contourArea(blob)
            if area < self._final_size_min:
                continue
            
            M = cv2.moments(blob)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                out_blobs.append((center,area))
            except ZeroDivisionError:
                pass
                
            color = np.uint8([[[(hsv_upper[0]+hsv_lower[0])/2,(hsv_upper[1]+hsv_lower[1])/2,(hsv_upper[2]+hsv_lower[2])/2]]])
        
            color = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)
            color = np.ndarray.tolist(color[0,0,:])
        
            cv2.circle(image_out, center, 4, (0,0,255), -1)
            cv2.drawContours(image_out, [blob], -1, color, 3)

        out_blobs = sorted(out_blobs, key = lambda b: b[1], reverse=True)
        rospy.loginfo_throttle(1,"Blobs visible: {}".format(len(out_blobs)))
            
        if self._disp:
            cv2.imshow("input",cv_image)
            cv2.imshow("output",image_out)
            cv2.waitKey(3)

        # Don't return anything if there are too many blobs in the scene
        if len(out_blobs)>5 or len(out_blobs)==0:
            target = None, None
            return target
        else:
            target = list(out_blobs[0][0])

        if self._invert:
            width, height, channels = cv_image.shape
            if target[0] > width/2:
                target[0] = 0
            else:
                target[0] = width
            if target[1] > height/2:
                target[1] = 0
            else:
                target[1] = height

        return target
    
        
if __name__=="__main__":
    rospy.init_node("blob_lookat")
    parser=argparse.ArgumentParser(description="Use opencv to get motion features from video")
    parser.add_argument('-d', '--display-video', help="Show the masked video on screen", action='store_true')
    parser.add_argument('-i', '--invert', help="Look away from objects instead of towards them", action='store_true')
    
    args = parser.parse_known_args()[0]
    
    c = BlobLookat(args.display_video, args.invert)
    rospy.spin()
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
import numpy
import math
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

from hlpr_lookat.lookat_opencv import CVLookat

HAAR_LOCATION = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/"
MIN_FACE_SIZE = 50
MAX_FACE_SIZE = 600

class FaceLookat(CVLookat):
    def __init__(self, display_on, delay_s):
        self.last = rospy.Time.now()
        self.delay = rospy.Duration(delay_s)
        CVLookat.__init__(self, display_on)
        self._front_face_cascade = cv2.CascadeClassifier(HAAR_LOCATION+'haarcascade_frontalface_default.xml')
        self._front_face_cascade2 = cv2.CascadeClassifier(HAAR_LOCATION+'haarcascade_frontalface_alt.xml')
        self._front_face_cascade3 = cv2.CascadeClassifier(HAAR_LOCATION+'haarcascade_frontalface_alt2.xml')
        self._side_face_cascade = cv2.CascadeClassifier(HAAR_LOCATION+'haarcascade_profileface.xml')

    def process_image(self, cv_image, display_on):
        if rospy.Time.now()-self.last < self.delay:
            rospy.logwarn_throttle(0.5, "Skipping frame")
            return None, None
        
        frontfaces1 = []
        frontfaces2 = []
        frontfaces3 = []
        sidefaces = []
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #gray = cv_image
        start = rospy.Time.now()
        frontfaces1 = self._front_face_cascade.detectMultiScale(gray, 1.3, 3, minSize=(MIN_FACE_SIZE,MIN_FACE_SIZE),maxSize=(MAX_FACE_SIZE,MAX_FACE_SIZE))
        #frontfaces2 = self._front_face_cascade2.detectMultiScale(gray, 1.3, 3,minSize=(MIN_FACE_SIZE,MIN_FACE_SIZE),maxSize=(MAX_FACE_SIZE,MAX_FACE_SIZE)) 
        #frontfaces3 = self._front_face_cascade3.detectMultiScale(gray, 1.3, 3, minSize=(MIN_FACE_SIZE,MIN_FACE_SIZE),maxSize=(MAX_FACE_SIZE,MAX_FACE_SIZE))
        #sidefaces = self._side_face_cascade.detectMultiScale(gray, 1.3, 3, minSize=(MIN_FACE_SIZE,MIN_FACE_SIZE),maxSize=(MAX_FACE_SIZE,MAX_FACE_SIZE))
        
        biggest_face_pos = (None,None)
        biggest_face_size = 0
        
        for (x,y,w,h) in frontfaces1:
            if w+h > biggest_face_size:
                biggest_face_pos = (x,y)
                biggest_face_size = (w+h)/2.0
        for (x,y,w,h) in frontfaces2:
            if w+h > biggest_face_size:
                biggest_face_pos = (x,y)
                biggest_face_size = (w+h)/2.0
        for (x,y,w,h) in frontfaces3:
            if w+h > biggest_face_size:
                biggest_face_pos = (x,y)
                biggest_face_size = (w+h)/2.0
        for (x,y,w,h) in sidefaces:
            if w+h > biggest_face_size:
                biggest_face_pos = (x,y)
                biggest_face_size = (w+h)/2.0

        face_x = biggest_face_pos[0]
        face_y = biggest_face_pos[1]
            
        if self._disp:
            for (x,y,w,h) in frontfaces1:
                cv2.rectangle(cv_image, (x,y), (x+w,y+w), (255,0,255),2)
            for (x,y,w,h) in frontfaces2:
                cv2.rectangle(cv_image, (x,y), (x+w,y+w), (255,255,0),2)
            for (x,y,w,h) in frontfaces3:
                cv2.rectangle(cv_image, (x,y), (x+w,y+w), (0,255,255),2)
            for (x,y,w,h) in sidefaces:
                cv2.rectangle(cv_image, (x,y), (x+w,y+w), (0,255,0),2)
            cv2.imshow("frame0",cv_image)
            cv2.waitKey(3)

        self.last=rospy.Time.now()
        return face_x, face_y

if __name__=="__main__":
    rospy.init_node("background_subtraction_features")
    parser=argparse.ArgumentParser(description="Use opencv to get motion features from video")
    parser.add_argument('-d', '--display-video', help="Show the masked video on screen", action='store_true')
    parser.add_argument('-t', '--delay-time', help="How long to wait before moving head again", type=float, default = 0.0)
    args = parser.parse_known_args()[0]
    
    c = FaceLookat(args.display_video, args.delay_time)
    rospy.spin()

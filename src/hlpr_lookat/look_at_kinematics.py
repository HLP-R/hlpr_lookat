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

from math import *

def clamp(x, limits):
  return max(min(x, limits[1]), limits[0])

#returns the plausible root of sin(x)*c1+cos(x)*c2 = c3
def trigSolution(params):
  y = atan(abs(params[1]/params[0]))
  h = params[2]/sqrt(params[0]*params[0]+params[1]*params[1])
  if(params[0] > 0 and params[1] < 0):
    return asin(h) + y
  if(params[0] < 0 and params[1] < 0):
    return asin(-h) - y
  if(params[0] < 0 and params[1] > 0):
    return asin(-h) + y
  else:
    return asin(h) - y

#TODO: pt_params from urdf?, limits from urdf?

#This only do kinematics wrt pan base. It is upto the programmer to provide the correct positions. This is a concious choice made by baris
class LookAtKin:
  def __init__(self, params=None, axes_mapping = None):

    #params: servo1 to servo2 up, servo2 to the right, up and forward 
    if params is None:
      #RGB_OPTICAL
      #self.pt_params = [-0.0985, 0.06175, -0.04201 ,0.0245]#[-0.0985, (0.03325+0.06175)/2, -0.04201 ,0.0245]
      #self.pt_params = [-0.0985, 0.095, -0.04201 ,0.0245]
      #MIDDLE
      #self.pt_params = [-0.0985, (0.03325+0.06175)/2, -0.04201 ,0.0245]
      #self.pt_params = [-0.0985, (0.095+0.0425)/2, -0.04201 ,0.0245]
      #IR_OPTICAL
      #self.pt_params = [-0.0985, 0.03325, -0.04201 ,0.0245]
      self.pt_params = [-0.0985, 0.0425, -0.04201 ,0.0245]

    #whether to calculate the joint position for the imaginary link at the end for ik
    self.calc_ik_im_trans = False

    self.pan_limits  = [-pi/3.,pi/2.]
    self.tilt_limits = [-pi/3., pi/3.]

    #mapping the axes orientations from kinect to base
    #robot to kin | kin to robot
    #  x   to  z  |  x  to  -y
    #  y   to -x  |  y  to  -z
    #  z   to -y  |  z  to   x
    if axes_mapping is None:
      self.axes_mapping_r2k = [[2,0,1],[1,-1,-1]]
    else:
      self.axes_mapping_r2k = axes_mapping

    self.axes_mapping_k2r = self._reverseAxesMapping(self.axes_mapping_r2k)
  
  #this class assumes that the coordinate frame is at the base of the pan/tilt unit AND this frame hass aligned with the kinect frame when pan = tilt = 0, meaining
  #z forward, x right and y down
  
  #given the object in the kinect frame, convert it to base frame
  def baseToObject(self, pos, theta, map_axes = True):
    X = (self.pt_params[1] + pos[0])*cos(theta[0]) + (self.pt_params[2] + pos[1])*sin(theta[0])*sin(theta[1]) - (self.pt_params[3] + pos[2])*sin(theta[0])*cos(theta[1])
    Y =  self.pt_params[0]                         + (self.pt_params[2] + pos[1])              *cos(theta[1]) + (self.pt_params[3] + pos[2])              *sin(theta[1])
    Z = (self.pt_params[1] + pos[0])*sin(theta[0]) - (self.pt_params[2] + pos[1])*cos(theta[0])*sin(theta[1]) + (self.pt_params[3] + pos[2])*cos(theta[0])*cos(theta[1])

    if map_axes:
      out_pos = self.mapAxes([X,Y,Z], self.axes_mapping_k2r)
    else:
      out_pos = [X,Y,Z]
    return out_pos

  def headFK(self, theta, distance_to_sensor = None):
    if distance_to_sensor is None:
      if len(theta)<3:
        distance_to_sensor = 1.
      else:
        distance_to_sensor = theta[2]
    return self.baseToObject([0,0,distance_to_sensor], theta[0:2])

  #given lookat_position in the pantilt base frame, returns the pan/tilt positions
  def headIK(self, lookat_position, map_axes = True):

    if map_axes:
      pos = self.mapAxes(lookat_position, self.axes_mapping_r2k)
    else:
      pos = lookat_position
      
    if pos[2] < 0:
      raise Exception("The robot cannot look back")
    t1 = trigSolution([pos[2], pos[0], self.pt_params[1]])
    t2 = trigSolution([sin(t1)*self.pt_params[1]-pos[2], cos(t1)*(pos[1]-self.pt_params[0]), cos(t1)*self.pt_params[2]])

    if t1 < self.pan_limits[0] or t1 > self.pan_limits[1]:
      #raise Exception("The pan joint is out of limits")
      print "The pan joint is out of limits"

    if t2 < self.tilt_limits[0] or t1 > self.tilt_limits[1]:
      #raise Exception("The tilt joint is out of limits")
      print "The tilt joint is out of limits"

    if self.calc_ik_im_trans:
      Dx = -pos[0] + (self.pt_params[1]*cos(t1) + self.pt_params[2]*sin(t1)*sin(t2))
      Dy =  pos[1] - (self.pt_params[0]         + self.pt_params[2]        *cos(t2))
      Dz =  pos[2] - (self.pt_params[1]*sin(t1) - self.pt_params[2]*cos(t1)*sin(t2))
      #This is the most computationally expensive way to calculate z3 but it avoids checking for div by 0
      #Also you do not even need to calculate this :)
      z3 = sqrt(Dx*Dx+Dy*Dy+Dz*Dz)-self.pt_params[3]
      return [t1,t2,z3]
    else:
      return [t1,t2]

  def mapAxes(self, in_pos, axes_mapping):
    pos = [0,0,0]
    for i in range(0,3):
      pos[axes_mapping[0][i]] = axes_mapping[1][i]*in_pos[i]

    return pos


  def _reverseAxesMapping(self,axes_mapping):
    reversed_mapping = [[0,0,0],[0,0,0]]
    for i in range(0,3):
      reversed_mapping[0][axes_mapping[0][i]] = i
      reversed_mapping[1][axes_mapping[0][i]] = axes_mapping[1][i]
    return reversed_mapping
    

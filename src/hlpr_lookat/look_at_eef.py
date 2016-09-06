#!/usr/bin/env python

import roslib
import rospy
import sys
import time
import tf
import math

from hlpr_lookat.srv import LookAt, LookAtT, LookAtTS
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Vector3

class LookAtEEF():
  def __init__(self):
    rospy.logwarn("Waiting for lookat service")
    rospy.wait_for_service('lookat_vec3')
    self.lookat = rospy.ServiceProxy('lookat_vec3', LookAt)
    rospy.logwarn("Lookat service loaded")
    rospy.Subscriber("eef_pose", Pose, self.poseCb)
    rospy.Subscriber("lookat_eef_toggle", Bool, self.toggleCb)
    self.toggle = False
    self.prevX = 0.0
    self.prevZ = 0.0

  def toggleCb(self, msg):
    self.toggle = msg.data

  def poseCb(self, msg):
    if not self.toggle:
      return

    pos = msg.position
    dx = pos.x - self.prevX
    dz = pos.z - self.prevZ
    dist = math.sqrt((dx ** 2) + (dz ** 2))
    if dist < 0.05:
      return
    print "dist: " + str(dist)
    self.prevX = pos.x
    self.prevZ = pos.z
    quat = msg.orientation
    orient = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
    self.eePose = [pos.x, pos.y, pos.z, orient[0], orient[1], orient[2]]
    print self.eePose
    self.lookatPos(self.eePose)

  def lookatPos(self, pos):
    if pos[0] < 0.9:
      return
    try:
      vec = Vector3()
      vec.x = pos[0]
      vec.y = pos[1]
      vec.z = pos[2]
      self.lookat(vec)
    except rospy.ServiceException, e:
      print "Lookat service call failed: %s"%e

if __name__== '__main__':
  rospy.init_node("hlpr_lookat_eef", anonymous=False)
  rospy.loginfo("Starting the eef lookat node")
  lookat = LookAtEEF()
  rospy.spin()

#!/usr/bin/env python

import roslib
import rospy
import sys
import time
import tf
import math

from hlpr_lookat.srv import LookAtTS, LookAtTSRequest
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, PoseStamped, Vector3

class LookAtFrame():
  def __init__(self):
    self.toggle = True
    self.frame = None

    lookat_topic = "/hlpr_lookat_s_tr"

    # a clean wait for service (no extraneous warnings)
    while not rospy.is_shutdown():
      try:
        rospy.wait_for_service(lookat_topic, 10)
        break
      except Exception as e:
        if rospy.is_shutdown():
          return
        rospy.logwarn("Waiting for lookat server")
    if rospy.is_shutdown():
      return

    self.lookat = rospy.ServiceProxy(lookat_topic, LookAtTS)
    rospy.loginfo("Lookat eef server has connected to lookat server.")

    rospy.Subscriber("lookat_frame_target", String, self.frame_cb, queue_size=1)
    rospy.Subscriber("lookat_frame_toggle", Bool, self.set_cb, queue_size=1)

    rospy.loginfo("Giving subscribers time to get set up...")
    rospy.sleep(0.5)

    rospy.loginfo("Starting server...")
    self.spin()

  def set_cb(self, msg):
    self.toggle = msg.data
    if self.toggle:
      rospy.loginfo("Tracking on.  Frame is {}".format(self.frame))
    else:
      rospy.loginfo("Tracking off.  Frame is {}".format(self.frame))

  def frame_cb(self, msg):
    self.frame = msg.data
    rospy.loginfo("Set to look at frame {}. Tracking is {}.".format(self.frame, self.toggle))

  def spin(self):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      if self.toggle and not self.frame is None and not len(self.frame)==0:
        req = LookAtTSRequest()
        req.desired_s_tr.child_frame_id = self.frame
        req.desired_s_tr.transform.rotation.w = 1.0
        self.lookat(req)
      r.sleep()
      
def main():
  rospy.init_node("hlpr_lookat_follow_frame")
  lookat = LookAtFrame()
  rospy.spin()


if __name__== '__main__':
  main()


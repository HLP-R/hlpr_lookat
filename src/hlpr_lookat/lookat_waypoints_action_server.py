#!/usr/bin/env python

# Copyright (c) 2016, Diligent Droids
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
# * Neither the name of hlpr_lookat nor the names of its
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

# Author: Vivian Chu, vchu@diligentdroids.com
'''
Action server that takes in an array of rospy durations and TransformStamped
and calls lookat service in the order received

See scan_scene.py as an example of how to send an action
'''

import actionlib
import rospy

from hlpr_lookat.msg import LookatWaypointsAction, LookatWaypointsGoal, LookatWaypointsResult
from hlpr_lookat.srv import LookAtTS

class LookatWaypoints:

	def __init__(self):

		# We want this to be an action client that gets called
		self.server = actionlib.SimpleActionServer('lookat_waypoints_action_server', LookatWaypointsAction, execute_cb=self.lookat_waypoints, auto_start=False)

		# This service expects a TransformStamped to execute
		self.lookat_service_name = 'lookat_s_tr' # Later can change to param if necessary
		
		# Wait for the lookat service - takes a Vector3 to lookat
		rospy.logwarn("Waiting for lookat service.")
		rospy.wait_for_service(self.lookat_service_name)
		self.lookat = rospy.ServiceProxy(self.lookat_service_name, LookAtTS)
		rospy.logwarn("Lookat service loaded")

		# Start the server
		self.server.start()

		# Do we need to load any parameters? 
		self.verbose = rospy.get_param("~verbose", False)

		# Create global result to return
		self.result = LookatWaypointsResult()

	def lookat_waypoints(self, goal):

		# Check if we have been preempted. 
		# Here for now - might make more sense in another spot
		while not self.server.is_preempt_requested():
	
			# Pull out from the goal the locations to scan
			scan_locs = goal.scan_positions
			time_durations = goal.scan_times

			# Check if the lengths match up - return error if not
			if len(scan_locs) != len(time_durations):
				self.result.success = False
				err_msg = "The scan positions and scan times do not match up"
				self.server.set_aborted(self.result, err_msg)
				rospy.logerr(err_msg)
				return	
			
			for i in xrange(len(scan_locs)):
				pos = scan_locs[i]
				time = time_durations[i]
				trans = pos.transform.translation
				if self.verbose:
					rospy.loginfo("Looking at location #:%d for %0.2f seconds. Values are (X: %0.2f, Y: %0.2f, Z: %0.2f) in frame: %s" % (i, time.to_sec(), trans.x, trans.y, trans.z, pos.child_frame_id))

				# Send the position command off
				try:
					response = self.lookat(pos)
					rospy.sleep(time)
				except rospy.ServiceException, e:
					rospy.logerr( "Lookat service call failed: %s" % e)
				
			self.result.success = True
			success_msg = "Finished looking at waypoints"
			self.server.set_succeeded(self.result, success_msg)
			rospy.loginfo(success_msg)
			return

		self.result.success = False
		abort_msg = "Looking at waypoints preempted" 
		self.server.set_aborted(self.result, abort_msg)
		rospy.loginfo(abort_msg)
		return
		

if __name__ == '__main__':

	rospy.init_node('scan_scene_action_server')
	LookatWaypoints()
	rospy.spin()

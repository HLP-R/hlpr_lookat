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

import actionlib
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from hlpr_lookat.msg import LookatWaypointsAction, LookatWaypointsGoal, LookatWaypointsResult


def transform_helper(vec3, frame):
    pos = TransformStamped()
    pos.child_frame_id = frame
    pos.header = Header()
    pos.transform = Transform()
    pos.transform.translation = vec3

    return pos

def main():

    # Connect to the action client
    scan_client = actionlib.SimpleActionClient('lookat_waypoints_action_server', LookatWaypointsAction)
    rospy.logwarn("Waiting for scan scene server to load")
    scan_client.wait_for_server()
    rospy.logwarn("Scan scene loaded")

    #Generate some positions
    up = transform_helper(Vector3(1.0,0.0,1.0), 'pan_base_link')
    down = transform_helper(Vector3(1.0,0.0,-2.0), 'pan_base_link')
    right = transform_helper(Vector3(1.0,2.0,0.2), 'pan_base_link')
    left = transform_helper(Vector3(1.0,-2.0,0.2), 'pan_base_link')
    left_up = transform_helper(Vector3(1.0,-2.0,1.0), 'pan_base_link')
    right_up = transform_helper(Vector3(1.0,2.0,1.0), 'pan_base_link')
    left_down = transform_helper(Vector3(1.0,-2.0,-1.0), 'pan_base_link')
    right_down = transform_helper(Vector3(1.0,2.0,-1.0), 'pan_base_link')

    # Generate some times
    pause_2 = rospy.Duration(2.0)
    time1 = rospy.Duration(2.0)
    time2 = rospy.Duration(2.0)
    time3 = rospy.Duration(2.0)
    time4 = rospy.Duration(2.0)

    # Store them away to send off
    positions = [left_up, up, right_up, right, right_down, down, left_down]
    scan_times = [pause_2, pause_2, pause_2, pause_2, pause_2, pause_2, pause_2]
 
    # Send the goals
    goal = LookatWaypointsGoal()
    goal.scan_positions = positions
    goal.scan_times = scan_times
    scan_client.send_goal(goal)

    # Print results
    rospy.loginfo("Waiting for scan to finish")
    scan_client.wait_for_result()
    rospy.loginfo(scan_client.get_goal_status_text())

if __name__ == '__main__':
    rospy.init_node('scan_scene')
    main()    

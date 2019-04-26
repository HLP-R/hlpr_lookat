#!/usr/bin/env python
'''Demo to turn detected faces into TFs at 1m distance'''

import rospy
from opencv_apps.msg import FaceArrayStamped
from geometry_msgs.msg import PoseStamped
import tf
import math

FOV_VERT = math.radians(58)
FOV_HORIZ = math.radians(87)
RES_VERT = 480.
RES_HORIZ = 640.

class Face2TF:
    def __init__(self):
        sub_topic = None
        for topic,ttype in rospy.get_published_topics():
            if "face_detection" in topic and topic[-5:] == "faces":
                sub_topic = topic
        if sub_topic is None:
            print "Error: no face topic found"
            exit()

        self.sub = rospy.Subscriber(sub_topic, FaceArrayStamped, self.to_tf_cb)
        self.broad = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

    def img_to_tf(self, img_x, img_y, name, frame):
        tf_z = 1.0

        
        angle_h = FOV_HORIZ*((img_x-(RES_HORIZ/2.))/(RES_HORIZ/2.))
        angle_v = FOV_VERT*((img_y-(RES_VERT/2.))/(RES_VERT/2.))

        tf_x = tf_z*math.tan(angle_h)
        tf_y = tf_z*math.tan(angle_v)

        rospy.loginfo("Broadcasting {} at ({},{},{}) [{}]".format(name,tf_x,tf_y,tf_z, frame))
        try:
            rospy.loginfo(
                "looking up transform from frame {} to frame {}".format(
                    frame, "base_link"))
            # Get position from the request
            pos = PoseStamped()
            pos.pose.position.x = tf_x
            pos.pose.position.y = tf_y
            pos.pose.position.z = tf_z
            pos.header.frame_id = frame
            pos.pose.orientation.x = 0
            pos.pose.orientation.y = 0
            pos.pose.orientation.z = 0
            pos.pose.orientation.w = 1
            

            # Lookup the transform between frames
            trans = self.listener.transformPose("base_link", pos)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Cannot look up translation, not moving head.")
            return False

        tf_x = trans.pose.position.x
        tf_y = trans.pose.position.y
        tf_z = trans.pose.position.z
        frame = "base_link"
        
        self.broad.sendTransform((tf_x,tf_y,tf_z),
                                 (0,0,0,1),
                                 rospy.Time.now(),
                                 name,
                                 frame)
        
    def to_tf_cb(self,msg):
        face_idx = 0
        for face in sorted(msg.faces, key= lambda face: (face.face.width+face.face.height)/2.):
            self.img_to_tf(face.face.x, face.face.y, "face{}".format(face_idx), msg.header.frame_id)
            face_idx += 1


        
        
if __name__=="__main__":
    rospy.init_node("face_track")
    Face2TF()
    rospy.spin()

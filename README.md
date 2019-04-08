## MIGRATION NOTE

This package is currently in the process being split into poli1 and poli2 versions. This is required for 3 reasons:
 - The two robots have different joint limits
 - The two robots have different base frame orientations
 - Poli1 still uses dynamixel msgs (for an old version of the dynamixel driver), but Poli2 has a new version of the driver that uses JointState (more standardized, but not backwards compatible with the old version of this package).



# HLP-R Lookat Package

Would you look at that!

This package has a few examples of how to do lookout related stuff. To get started look at the lookat_server.py. To run the service, just run it with `python lookat_server.py` for now. The package will be configured to do `rosrun` at some point.

When you run it, it creates 3 services: 

1. lookat_vec3: Expects a geometry_msgs/Vector3 in base_link frame to look at

2. lookat_tr: Expects a geometry_msgs/Transform in base_link frame to look at

3. lookat_s_tr: Expects a geometry_msgs/TransformStamped with a valid frame_id (i.e. something that exists in the tf tree) to look at.

The look_at_client.py file has some example code to look at the end effector using the service.

The follow_eef.py file has some example code to look at things without using the service. 



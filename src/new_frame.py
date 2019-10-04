#!/usr/bin/env python

import rospy
import tf
import sys


if __name__ == "__main__":
	rospy.init_node('pcd_frame_broadcaster')

	tf_br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	
	while not rospy.is_shutdown():
		print("TransformBroadcaster Running	")
		
		tf_br.sendTransform((0.0,0.0,0.0),(0.0,0.0,0.0,1.0),rospy.Time.now(),"map",sys.argv[1])
		
		rate.sleep()
		
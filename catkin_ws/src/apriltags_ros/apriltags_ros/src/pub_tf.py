#!/usr/bin/env python
import roslib
import rospy
import tf
from apriltags_ros.msg import AprilTagDetectionArray

def main():
	apriltag_sub = rospy.Subscriber("/sr300/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size=1)
	rospy.spin()

def apriltag_callback(data):
	if len(data.detections) >=1:
		br = tf.TransformBroadcaster()
		for detect in data.detections:
			if(detect.id == 0):
				id = detect.id
				break
			id = detect.id
		if id == 0:
			br.sendTransform((0.0, 0.0, 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"/new_tf","/viewed_tag_0")
		elif id == 1:
			br.sendTransform((-0.166, 0.248, 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"/new_tf","/viewed_tag_1")
		#print id

if __name__=='__main__':
	rospy.init_node('tf_broadcaster', anonymous=True)
	main()
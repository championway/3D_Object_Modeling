#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm
import math
from apriltags_ros.msg import AprilTagDetectionArray
from demos.msg import pose


rospy.init_node('pose_show', anonymous=True)
def main():
	apriltag_sub = rospy.Subscriber("/123/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size=1)
	rospy.sleep(2)
	rospy.spin()

def apriltag_callback(data):
	#return degree in radians
        pose_result = pose()
        apriltag_pub = rospy.Publisher("/pose_output", PoseList, queue_size=1)
	if len(data.detections) >=1:
		detection = data.detections[0]
		#print detection.pose
		poselist = posetoposelist(detection.pose.pose)
		#print detection.pose.pose.position.x
		orientation = poselist[3:7]
		angle = quaternion_to_euler_angle(orientation)
		#print angle, "\n"
		pose_result.PoseList = [poselist[0], poselist[1], poselist[2], angle[0], angle[1], angle[2]]
		#print pose_result
		apriltag_pub.publish(pose_result)
                return pose_result
	else:
		print "Detect failed"
		apriltag_pub.publish([])
                return ["Failed"]

def posetoposelist(pose):
	return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def quaternion_to_euler_angle(pose):
	y_sqr = pose[1] * pose[1]
	t0 = +2.0 * (pose[3]*pose[0] + pose[1]*pose[2])
	t1 = +1.0 - 2.0*(pose[0]*pose[0] + y_sqr)
	X = math.atan2(t0,t1)

	t2 = +2.0 * (pose[3]*pose[0] - pose[1]*pose[2])
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.asin(t2)

	t3 = +2.0 * (pose[3]*pose[2] + pose[0]*pose[1])
	t4 = +1.0 - 2.0 * (y_sqr + pose[2]*pose[2])
	Z = math.atan2(t3, t4)
	return X, Y, Z

if __name__=='__main__':
	main()


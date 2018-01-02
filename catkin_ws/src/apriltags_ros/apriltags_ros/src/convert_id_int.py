#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from apriltags_ros.msg import AprilTagDetectionArray

rospy.init_node('convert_id_int', anonymous=True)
def main():
	apriltag_sub = rospy.Subscriber("/sr300/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size=1)
	rospy.spin()

def apriltag_callback(data):
	if len(data.detections) >=1:
		id = Image()
		for detect in data.detections:
			id.header = detect.pose.header
			if(detect.id == 0):
				id.step = detect.id
				break
			id.step = detect.id
		#print id.data
		#print " "
		id_pub = rospy.Publisher("/id_output", Image, queue_size=1)
		id_pub.publish(id)
	
if __name__=='__main__':
	main()


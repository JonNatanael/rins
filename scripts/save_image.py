#!/usr/bin/env python
import roslib
#roslib.load_manifest('facedetector')
import rospy 
import numpy as np
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA
import sensor_msgs.msg
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3, PointStamped
from tf import TransformListener


def image_callback(self, image, camera):
#def image_callback(self, image):
	#we need to bringup minimal.launch
	#we need to bringup 3dsensor.launch
	#amcl_demo.launch
	#localizer/localizer
	#rosrun mas_server map_server map/map.yaml

	camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/camera_info')
	self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)

	try:
		cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
	except CvBridgeError, e:
		print e






# Main function.    
if __name__ == '__main__':

		rospy.init_node('image_saver')
		try:
			#fd = CyllinderDetector()
			rospy.spin()	
		except rospy.ROSInterruptException:
			print "Shutting down"
		
		cv2.destroyAllWindows()

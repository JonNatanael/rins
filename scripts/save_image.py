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
import random
import string


class ImageSaver():

	def image_callback(self,image, camera):
	#def image_callback(self, image):
		#we need to bringup minimal.launch
		#we need to bringup 3dsensor.launch
		#amcl_demo.launch
		#localizer/localizer
		#rosrun mas_server map_server map/map.yaml

		#print image
		N = 15

		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")	
			#print cv_image		
			cv2.imshow("Image window", cv_image)
			key = cv2.waitKey(3)
			if key==10:
				name = ''.join(random.choice(string.ascii_uppercase + string.digits + string.ascii_lowercase) for _ in range(N))
				name ='images/'+name+'.jpg'
				print name
				cv2.imwrite(name, cv_image)

		except CvBridgeError, e:
			print e


	def __init__(self):
		cv2.namedWindow("Image window", 1)
		self.bridge = CvBridge()

		image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_color')
		camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/camera_info')		

		

		self.image_sub = message_filters.Subscriber(image_topic, Image)
		self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
		self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub], 10)
		self.joined_sub.registerCallback(self.image_callback)





# Main function.    
if __name__ == '__main__':

		print "Starting.."
		rospy.init_node('image_saver')
		try:
			fd = ImageSaver()
			rospy.spin()	
		except rospy.ROSInterruptException:
			print "Shutting down"
		
		cv2.destroyAllWindows()

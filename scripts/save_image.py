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
from facedetector.msg import Detection

class ImageSaver():

	def image_callback(self,image, camera, faces):
	#def image_callback(self, image):
		#we need to bringup minimal.launch
		#we need to bringup 3dsensor.launch
		#amcl_demo.launch
		#localizer/localizer
		#rosrun map_server map_server map/map.yaml

		#print image
		N = 15
		print faces.x[0],faces.y[0]

		try:
			for i in xrange(0,len(faces.x)):		
				cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")	
				#print cv_image		
				cv_image=cv_image[faces.y[i]:faces.y[i]+faces.height[i],faces.x[i]:faces.x[i]+faces.width[i]]
				cv2.imshow("Image window", cv_image)
				key = cv2.waitKey(1)
				if key==10:
					name = ''.join(random.choice(string.ascii_uppercase + string.digits + string.ascii_lowercase) for _ in range(N))
					#name ='images/'+name+'.jpg'

					name = '/home/team_beta/ROS/src/rins/images/'+name+'.jpg'
					print name
					#try:
					print cv2.imwrite(name, cv_image)
					#except e:
					#		print "saving failed"

		except CvBridgeError, e:
			print e
		#rospy.sleep(1)


	def __init__(self):
		cv2.namedWindow("Image window", 1)
		self.bridge = CvBridge()

		image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_color')
		camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/camera_info')		
		faces_topic = rospy.get_param('~faces_topic', '/facedetector/faces')

		self.faces_sub = message_filters.Subscriber(faces_topic, Detection)
		self.image_sub = message_filters.Subscriber(image_topic, Image)
		self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
		self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub, self.faces_sub], 20)
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

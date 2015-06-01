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
from geometry_msgs.msg import Point, Vector3

# Node for cyllinder detection.
class CyllinderDetector():
	
	
						 #[H/2,   S,  V]	
	#lower_red = np.array([150, 80,   0],dtype=np.uint8)
	#upper_red = np.array([180, 255,255],dtype=np.uint8)
	boundaries = [
	([165, 80,  0], [180, 255,255]), #red
	([35, 50,  0], [70, 255,255]), #green
	([15, 100,  0], [35, 255,255]), #yellow
	([85, 35,  80], [150, 255,255])	#blue
	]

	colors = [
	[0, 0,  255], #red
	[0, 255,  0], #green
	[0, 255,255], #yellow
	[255, 0,  0]	#blue
	]

	def image_callback(self, image, camera):
	#def image_callback(self, image):
		#we need to bringup minimal.launch
		#we need to bringup 3dsensor.launch

		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError, e:
			print e

		camera_model = PinholeCameraModel()
		camera_model.fromCameraInfo(camera)

		(height,width,channels) = cv_image.shape
		hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV) #convert to hsv for analisys
		
		i=0

		for (lower, upper) in self.boundaries: #parse all boundaries
			# create np arrays from the boundaries
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")

			# find the colors within the specified boundaries and apply the mask
			mask = cv2.inRange(hsv, lower, upper)
			#output = cv2.bitwise_and(hsv, hsv, mask = mask)

			#create the wanted kernel
			# kernel_size = 20
			# kernel = np.ones((kernel_size, kernel_size)) / (kernel_size*kernel_size)
			
			#apply the filter / buch of filters
			#cv2.filter2D(src, ddepth, kernel[, dst[, anchor[, delta[, borderType]]]])
			#blur = cv2.filter2D(mask, -1, kernel, anchor = (-1, -1), delta = 1)
			#mask_cleaned = cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11,0)
			#ret3, mask_cleaned = cv2.threshold(blur, (kernel_size*kernel_size)/2, 255, cv2.THRESH_BINARY)
			kernel = np.ones((3,3),np.uint8)
			mask_cleaned = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel, iterations = 4)

			cont_img = mask_cleaned.copy()
			contours, hierarchy = cv2.findContours(cont_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			cyllinderContour = np.array([]);
			cArea = 0
			for cnt in contours:
				area = cv2.contourArea(cnt)

				if area < 3000: #minimal size 
					continue
				if len(cnt) < 5:
				 	continue
				if cArea < area: #if we don't have a bigger field selected already
					ell = cv2.fitEllipse(cnt)

					u = int(ell[0][0])
					v = int(ell[0][1])
					point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
					         ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)
					resp = self.localize(image.header, point, 10)
					#if ell[0][1] > (2 * height/3):
					if resp:
						if resp.pose.position.y == 0 and resp.pose.position.x == 0 and resp.pose.position.y == 0:
							continue
						if abs(resp.pose.position.y) < 0.2:
							cyllinderContour = cnt
							cArea = area
			
			#if cArea > 0:
			#	print cArea

			if cyllinderContour.any():
				ellipse = cv2.fitEllipse(cyllinderContour)
				#print "Center:   " 
				#print ellipse[0]
				cv2.ellipse(cv_image, ellipse, self.colors[i], 2)
				cv2.drawContours(cv_image, [cyllinderContour], -1, self.colors[i]) 
			i+=1
		
		#output = cv2.bitwise_and(cv_image, cv_image, mask = mask_cleaned) #display original image masked with the provided parameters
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)

		# n = len(faces.x)

		# markers = MarkerArray()

		# for i in xrange(0, n):
		# 	u = faces.x[i] + faces.width[i] / 2
		# 	v = faces.y[i] + faces.height[i] / 2
		# 	#print u, v
		# 	point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
		#          ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)
		# 	#print point
		# 	resp = self.localize(faces.header, point, 3)
		# 	if resp:
		# 		marker = Marker()
		# 		marker.header.stamp = faces.header.stamp
		# 		marker.header.frame_id = faces.header.frame_id
		# 		marker.pose = resp.pose
		# 		marker.type = Marker.CUBE
		# 		marker.action = Marker.ADD
		# 		marker.frame_locked = False
		# 		marker.lifetime = rospy.Time(0)
		# 		marker.id = i
		# 		marker.scale = Vector3(0.1, 0.1, 0.1)
		# 		marker.color = ColorRGBA(1, 0, 0, 1)
		# 		markers.markers.append(marker)

		# self.markers_pub.publish(markers)

		self.message_counter = self.message_counter + 1


	def __init__(self):
		#laufat mora rosrun usb_camera usb_cam_node za debuganje preko webCama
		#drugace rabimo pa kinect prizgan

		region_scope = rospy.get_param('~region', 3)
		markers_topic = rospy.get_param('~markers_topic', rospy.resolve_name('%s/markers' % rospy.get_name()))
		image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_color') #kinect
		#image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw') #webcam
		camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/camera_info')		

		rospy.wait_for_service('localizer/localize')
		self.localize = rospy.ServiceProxy('localizer/localize', Localize)

		cv2.namedWindow("Image window", 1) #Control window
		self.bridge = CvBridge()

		#self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
		self.image_sub = message_filters.Subscriber(image_topic, Image)
		self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
		self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub], 10)
		self.joined_sub.registerCallback(self.image_callback)

		self.markers_pub = rospy.Publisher(markers_topic, MarkerArray, queue_size=10)

		self.message_counter = 0

# Main function.    
if __name__ == '__main__':

		rospy.init_node('cyllinderdetector')
		try:
			fd = CyllinderDetector()
			rospy.spin()	
		except rospy.ROSInterruptException:
			print "Shutting down"
		
		cv2.destroyAllWindows()

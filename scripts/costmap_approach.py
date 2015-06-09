#!/usr/bin/env python
import roslib
import rospy, math
import numpy as np
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA
import sensor_msgs.msg
import message_filters
import cv2, cv
from cv_bridge import CvBridge, CvBridgeError
from localizer.srv import Localize
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, PointStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from tf import TransformListener

# Node for cyllinder detection.
class CostmapApproach():

	width = -1
	height = -1 
	map_metadata = None
	base_costmap = np.array([])
	costmap = np.array([])
	locations = []
	f_app = None
	cy_app = None

	def costmap_callback(self, costmap):
		self.width = costmap.info.width
		self.height = costmap.info.height
		self.map_metadata = costmap.info

		np_arr = []
		for x in costmap.data:
			if x < 0:
				np_arr.append(0)
			else:
				np_arr.append(x)

		np_arr = np.reshape(np_arr, (self.height, self.width))
		cv_image = np.asarray(np_arr, np.uint8)
		cv_image = cv2.flip(cv_image,0)
		#cv_image = (255-cv_image)

		self.base_costmap = cv_image
		#print costmap.data
		#print np_arr
		#cv2.imshow("Initial costmap", cv_image)
		#cv2.waitKey(50)

	def update_callback(self, update):

		np_arr = []
		for x in update.data:
			if x < 0:
				np_arr.append(0)
			else:
				np_arr.append(x)

		np_arr = np.reshape(np_arr, (update.height, update.width))
		cv_image = np.asarray(np_arr, np.uint8)
		cv_image = cv2.flip(cv_image,0)
		#cv_image = (255-cv_image)

		img = self.base_costmap
		y = self.height - update.y - update.height
		img[ y:(y+update.height), update.x:(update.x+update.width)] = cv_image

		#print costmap.data
		#print np_arr
		self.costmap = img

		self.refresh_costmap()

	def clicked_callback(self, stamped_point):
		point = self.point_on_map(stamped_point.point)
		print "Pixel added to Points", point.x, point.y
		
		self.locations.append([x , y])
		self.refresh_costmap()

	def cylinders_callback(self, marker_array):
		self.cy_app = MarkerArray()
		for marker in marker_array.markers
			point = self.point_on_map(marker.position)
			orientation = self.quaternion_to_look_from_to(point, marker.pose.position)

			marker.scale = Vector3(marker.scale.x, marker.scale.y, 0)
			marker.pose.position = point
			marker.pose.orientation = orientation
			marker.type = 0 #arrow
			self.cy_app.markers.append(marker)
			#print "Cylinder added to Points", point.x, point.y
		self.refresh_costmap()

	def faces_callback(self, marker_array):
		self.f_app = MarkerArray()
		for marker in marker_array.markers
			point = self.point_on_map(marker.pose.position)
			orientation = self.quaternion_to_look_from_to(point, marker.pose.position)

			marker.scale = Vector3(marker.scale.x, marker.scale.y, 0)
			marker.pose.position = point
			marker.pose.orientation = orientation
			marker.type = 0 #arrow
			self.f_app.markers.append(marker)
			#print "Face added to Points", point.x, point.y
		self.refresh_costmap()

	def point_on_map(self, point):
		x = -self.map_metadata.origin.position.x/self.map_metadata.resolution  +  point.x/self.map_metadata.resolution
		x = int(round(x))
		y = -self.map_metadata.origin.position.y/self.map_metadata.resolution  +  point.y / self.map_metadata.resolution
		y = self.height - int(round(y)) 
		return Point(x, y, 0)

	def quaternion_to_look_from_to(self, from_point, to_point):
		delta_x = to_point.x - from_point.x
		delta_y = -( to_point.y - from_point.y ) #y je obrnjen
		theta = np.arctan( delta_y/delta_x )
		return Quaternion(0, 0, sin(theta), cos(theta))

		
	def calculate_approach(self, x, y, diameter):
		scan = np.zeros((self.width, self.height),np.uint8)
		cv2.circle(scan, (x,y), diameter/2, 255, diameter)
		points = np.transpose(np.where(scan==255)) #we get array of coordinates where the circle is
												  #in a [y, x] manner
		min_coord = []
		min_value = 50	#map value treshold, what is acceptable
					#127 is definitely not in collision, as per http://wiki.ros.org/costmap_2d
					#we could scale the color balance graph of the image or just scale 0-255 on 0-100
		for coord in points:
			cur_value = self.costmap[coord[0]][coord[1]] #costmap value of one of the points on the circle
			if (cur_value < min_value):
				min_value = cur_value
				min_coord = coord

		if len(min_coord) > 0:
			#print "We found a suitable approach:",  min_coord
			return min_coord
		else:
			#print "No suitable approach"
			return []

	def refresh_costmap(self):

		color_img = cv2.cvtColor(self.costmap, cv.CV_GRAY2RGB)
		max_scan_size = 10
		for x in self.locations: #clicked points
			cv2.circle(color_img, (x[0], x[1]), max_scan_size, (150,150,150))
			
			appc = []
			scansize = 1
			while (len(appc) == 0) and scansize < max_scan_size:
				appc = self.calculate_approach(x[0], x[1], scansize)
				scansize+=1

			if len(appc) > 0:
				cv2.circle(color_img, (appc[1], appc[0]), 1, (0,255,0), 2)
		
		for f in self.f_app: #face approach markers
			cv2.circle(color_img, (f.pose.position.x, f.pose.position.y), max_scan_size, f.color / 2)
			
			appc = []
			scansize = 1
			while (len(appc) == 0) and scansize < max_scan_size:
				appc = self.calculate_approach(f.pose.position.x, f.pose.position.y, scansize)
				scansize+=1

			if len(appc) > 0:
				cv2.circle(color_img, (appc[1], appc[0]), 1, f.color, 2)


		cv2.imshow("Enhanced costmap", color_img)
		cv2.waitKey(3)

	def __init__(self):
		costmap_topic = rospy.get_param('~costmap_topic', '/move_base/global_costmap/costmap')
		update_topic = rospy.get_param('~update_topic', '/move_base/global_costmap/costmap_updates')
		#costmap handling
		try:
			self.costmap_sub = rospy.Subscriber(costmap_topic, OccupancyGrid, self.costmap_callback)
			rospy.wait_for_message(costmap_topic, OccupancyGrid, timeout=2)
		except(rospy.ROSException), e:
			print "Costmap topic not available, aborting..."
			print "Error message: ", e
		print "Got costmap"
		self.update_sub = rospy.Subscriber(update_topic, OccupancyGridUpdate, self.update_callback)
		
		#data input
		clicked_topic = rospy.get_param('~clicked_topic', '/clicked_point')
		self.clicked_sub = rospy.Subscriber(clicked_topic, PointStamped, self.clicked_callback)

		cylinders_topic = rospy.get_param('~cylinders_topic', 'cylinder_detector/markers')
		self.cylinders_sub = rospy.Subscriber(cylinders_topic, MarkerArray, self.cylinders_callback)

		faces_topic = rospy.get_param('~faces_topic', 'faces_detector/markers')
		self.faces_sub = rospy.Subscriber(faces_topic, MarkerArray, self.faces_callback)

		#data output
		app_faces_topic = rospy.get_param('~app_faces_topic', rospy.resolve_name('%s/faces' % rospy.get_name()))
		self.app_faces_pub = rospy.Publisher(app_faces_topic, MarkerArray, queue_size=10)

		app_cylinders_topic = rospy.get_param('~app_cylinders_topic', rospy.resolve_name('%s/cylinders' % rospy.get_name()))
		self.app_cy_pub = rospy.Publisher(app_cylinders_topic, MarkerArray, queue_size=10)


		self.bridge = CvBridge()

		self.message_counter = 0

		print "Waiting for callbacks..."

# Main function.    
if __name__ == '__main__':

		rospy.init_node('cyllinderdetector')
		try:
			fd = CostmapApproach()
			rospy.spin()	
		except rospy.ROSInterruptException:
			print "Shutting down"
		
		cv2.destroyAllWindows()

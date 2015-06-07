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
	approaches = []

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
		x = -self.map_metadata.origin.position.x/self.map_metadata.resolution  +  stamped_point.point.x/self.map_metadata.resolution
		x = int(round(x))
		y = -self.map_metadata.origin.position.y/self.map_metadata.resolution  +  stamped_point.point.y / self.map_metadata.resolution
		y = self.height - int(round(y)) 
		print "Pixel added to Points", [x, y]
		self.locations.append([x , y])
		self.refresh_costmap()
		
	def calculate_approach(self, x, y, diameter):
		scan = np.zeros((self.width, self.height),np.uint8)
		cv2.circle(scan, (x,y), diameter/2, 255, diameter)
		points = np.transpose(np.where(scan==255)) #we get array of coordinates where the circle is
												  #in a [y, x] manner
		min_coord = []
		min_value = 95	#map value treshold, what is acceptable
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
		for x in self.locations:
			cv2.circle(color_img, (x[0], x[1]), 6, (255,255,255))
			
			appc = []
			scansize = 1
			while (len(appc) == 0) and scansize < 6:
				appc = self.calculate_approach(x[0], x[1], scansize)
				scansize+=1

			if len(appc) > 0:
				cv2.circle(color_img, (appc[1], appc[0]), 1, (0,255,0))
		cv2.imshow("Enhanced costmap", color_img)
		cv2.waitKey(3)

	def __init__(self):
		costmap_topic = rospy.get_param('~costmap_topic', '/move_base/global_costmap/costmap')
		update_topic = rospy.get_param('~update_topic', '/move_base/global_costmap/costmap_updates')
		clicked_topic = rospy.get_param('~clicked_topic', '/clicked_point')

		try:
			self.costmap_sub = rospy.Subscriber(costmap_topic, OccupancyGrid, self.costmap_callback)
			rospy.wait_for_message(costmap_topic, OccupancyGrid, timeout=2)
		except(rospy.ROSException), e:
			print "Costmap topic not available, aborting..."
			print "Error message: ", e
		print "Got costmap"

		self.update_sub = rospy.Subscriber(update_topic, OccupancyGridUpdate, self.update_callback)
		self.clicked_sub = rospy.Subscriber(clicked_topic, PointStamped, self.clicked_callback)


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

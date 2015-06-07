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
	base_costmap = np.array([])

	def costmap_callback(self, costmap):
		self.width = costmap.info.width
		self.height = costmap.info.height

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

		print "Got costmap", self.width, self.height

	def update_callback(self, update):
		if self.width < 0 or self.height < 0:
			print "Costmap update blocked"
			return None

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
		img[ y : y+update.height, update.x:update.x+update.width] = cv_image

		#print costmap.data
		#print np_arr
		cv2.imshow("Costmap update", img)
		cv2.waitKey(3)

	def __init__(self):
		costmap_topic = rospy.get_param('~costmap_topic', '/move_base/global_costmap/costmap')
		update_topic = rospy.get_param('~update_topic', '/move_base/global_costmap/costmap_updates')

		self.costmap_sub = rospy.Subscriber(costmap_topic, OccupancyGrid, self.costmap_callback)
		self.update_sub = rospy.Subscriber(update_topic, OccupancyGridUpdate, self.update_callback)

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

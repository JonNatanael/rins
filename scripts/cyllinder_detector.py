#!/usr/bin/env python
import roslib
import rospy, math
import numpy as np
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA, Empty
import sensor_msgs.msg
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3, PointStamped
from nav_msgs.msg import OccupancyGrid
from tf import TransformListener
from datetime import datetime


# Node for cyllinder detection.
class CyllinderDetector():
	
	
						 #[H/2,   S,  V]	
	#lower_red = np.array([150, 80,   0],dtype=np.uint8)
	#upper_red = np.array([180, 255,255],dtype=np.uint8)
	boundaries = [
	([165, 80,  30], [180, 255,255]), #red
	([35, 80,  30], [70, 255,255]), #green
	([15, 100,  30], [35, 255,255]), #yellow
	([85, 60,  60], [150, 255,255])	#blue
	]

	colors = [
	[0, 0,  255], #red
	[0, 255,  0], #green
	[0, 255,255], #yellow
	[255, 0,  0]	#blue
	]

	tags = []
	markers_by_color = [[],[],[],[]]
	color_names = ["red", "green", "yellow", "blue"]

	all_cyllinders = None

	map_metadata = None
	bloated_map = np.array([])

	def image_callback(self, image, camera):
	#def image_callback(self, image):
		#we need to bringup minimal.launch
		#we need to bringup 3dsensor.launch
		#rins amcl_demo.launch this launch is in conflict with 3dsensor
		#localizer/localizer
		#rosrun map_server map_server map/map.yaml

		#print "Got callback", image.header.stamp

		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError, e:
			print e

		camera_model = PinholeCameraModel()
		camera_model.fromCameraInfo(camera)

		(height,width,channels) = cv_image.shape
		hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV) #convert to hsv for analisys
		
		i=0

		for (lower, upper) in self.boundaries: #parse all colors
			
			#Find contours of each color
			cyllinderContour = self.findCyllinderContour(hsv, lower, upper, image, camera_model)
			if cyllinderContour.any(): #we got a hit!!!
				ellipse = cv2.fitEllipse(cyllinderContour)
				
				marker = self.markerFromCoutourEllipse(ellipse, i, image, camera_model)
				if marker:
					if len(self.markers_by_color[i]) > 3000: #lets not clog the memory and burden the clusterer shall we
						del self.markers_by_color[i][0:100]  #purge old markers
					self.markers_by_color[i].append(marker)

				cv2.ellipse(cv_image, ellipse, self.colors[i], 2)
				cv2.drawContours(cv_image, [cyllinderContour], -1, self.colors[i]) 

			i+=1

		# if self.message_counter % 100 == 0: #calculate clusters every 100 messages...about 4s?
		# 									#this is VERY expensive
		# 	self.all_cyllinders = MarkerArray()
		# 	#print " "
		# 	#print "Counter:", self.message_counter
		# 	for x in range(len(self.markers_by_color)): #iterate through markers of all colors
		# 		print "Current color:", self.color_names[x], "markers in bag:", len(self.markers_by_color[x])
		# 		if (len(self.markers_by_color[x]) == 0):
		# 			continue

		# 		#time = datetime.now()
		# 		clusters = self.DBSCAN_markers(self.markers_by_color[x], 0.1, 20)
		# 		#print "DBSCAN took ", datetime.now() - time

		# 		center = []
		# 		if len(clusters) > 0:
		# 			center = self.centerOfProminentCluster(clusters)
		# 			#print center

		# 		if len(center) > 0:
		# 			mkr = self.markers_by_color[x][0] #take random properly colored marker
		# 			mkr.id = x #we give it the ID of its color IDX
		# 			mkr.pose.position.x = center[0]
		# 			mkr.pose.position.y = center[1]
		# 			mkr.pose.position.z = 0.2
		# 			mkr.scale = Vector3(0.24, 0.24, 0.4)
		# 			mkr.type = Marker.CYLINDER
		# 			self.all_cyllinders.markers.append(mkr)
		# 		#self.all_cyllinders.markers += self.markers_by_color[x] #add whole color to all_cyllinders
				
		# 	if self.all_cyllinders:
		# 		self.markers_pub.publish(self.all_cyllinders)

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)

		self.message_counter = self.message_counter + 1

	def calculateCluster(self):
		print "start calculating cylinder clusters"
		self.all_cyllinders = MarkerArray()
		for x in range(len(self.markers_by_color)): #iterate through markers of all colors
			print "Current color:", self.color_names[x], "markers in bag:", len(self.markers_by_color[x])
			if (len(self.markers_by_color[x]) == 0):
				continue
			clusters = self.DBSCAN_markers(self.markers_by_color[x], 0.1, 20)

			center = []
			if len(clusters) > 0:
				center = self.centerOfProminentCluster(clusters)

			if len(center) > 0:
				mkr = self.markers_by_color[x][0] #take random properly colored marker
				mkr.id = x #we give it the ID of its color IDX
				mkr.pose.position.x = center[0]
				mkr.pose.position.y = center[1]
				mkr.pose.position.z = 0.2
				mkr.scale = Vector3(0.24, 0.24, 0.4)
				mkr.type = Marker.CYLINDER
				self.all_cyllinders.markers.append(mkr)
			
		self.markers_pub.publish(self.all_cyllinders)
		print "calculating cylinder clusters finished"

	def markerFromCoutourEllipse(self, ellipse, color_idx, image, camera_model):
		u = int(ellipse[0][0])
		v = int(ellipse[0][1])
		point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
			         ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)
		resp = self.localize(image.header, point, 10)

		#print resp

		try:
			resp.pose.position.z += 0.12 # r/2 of the cyllinder added to the depth
			mkr = self.makeMarker(resp.pose, self.colors[color_idx], self.message_counter * len(self.colors) +  color_idx )

			ps = PointStamped()
			ps.header.stamp = rospy.Time()
			ps.header.frame_id = "camera_rgb_optical_frame"
			ps.point = mkr.pose.position
			p = self.listener.transformPoint("map", ps)

			mkr.header.frame_id = "map"
			mkr.pose.position = p.point
			return mkr

		except Exception as ex:
			print "ERROR"
			print ex
			return None

	def findCyllinderContour(self, hsv, lower, upper, image, camera_model):
		# create np arrays from the boundaries
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")

			# find the colors within the specified boundaries and apply the mask
			mask = cv2.inRange(hsv, lower, upper)
			#output = cv2.bitwise_and(hsv, hsv, mask = mask)
			
			#apply the filter / buch of filters
			kernel = np.ones((3,3),np.uint8)
			mask_cleaned = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel, iterations = 4)
			mask_cleaned = cv2.morphologyEx(mask,cv2.MORPH_CLOSE, kernel, iterations = 2)
			mask_cleaned = cv2.dilate(mask,kernel,iterations = 1)

			cont_img = mask_cleaned.copy()
			contours, hierarchy = cv2.findContours(cont_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			cyllinderContour = np.array([]);
			max_contour_area = 0
			for cnt in contours:
				area = cv2.contourArea(cnt)

				if area < 3000: #minimal size on image
					continue
				if len(cnt) < 5:
				 	continue
				if max_contour_area < area: #if we don't have a bigger field selected already
					if self.contourIsValidContender(cnt, camera_model, image.header):
						#print "Added", resp.pose.position
						cyllinderContour = cnt
						max_contour_area = area
			
			#if cArea > 0:
			#	print cArea

			return cyllinderContour

	def contourIsValidContender(self, cnt, camera_model, header):
		ell = cv2.fitEllipse(cnt)
		u = int(ell[0][0]) #get the center of the elipse
		v = int(ell[0][1])
		point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
		         ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)
		resp = self.localize(header, point, 10)

		#if ell[0][1] > (2 * height/3):
		if resp and ell[0][1] > 0 and ell[0][0] > 0:
			#check if the response is valid. Cmon, it's never gonna be exactly 0,0,0, REALLY
			if resp.pose.position.y == 0 and resp.pose.position.x == 0 and resp.pose.position.y == 0:
				#print "Too close", ell[0]
				return False #response for object too close/too far
			elif self.pointIsInsideMap(resp):
				return True

		return False

	def pointIsInsideMap(self, localizerResponse):
		resp = localizerResponse
		try:
			resp.pose.position.z += 0.12 # r/2 of the cyllinder added to the depth
			ps = PointStamped()
			ps.header.stamp = rospy.Time()
			ps.header.frame_id = "camera_rgb_optical_frame"
			ps.point = resp.pose.position
			p = self.listener.transformPoint("map", ps)

			#filters
			if p.point.z > 0.4: #above cylinder height
				return False
			if p.point.z < 0: #ehm...underground?
				return False

			#print p.point
			if self.mappixelOfPoint(p.point) < 255 : #playfield is absolutely white
				return False

			return True

		except Exception as ex:
			print "ERROR in pointIsInsideMap", ex
			return False

	def mappixelOfPoint(self,point):
		x = -self.map_metadata.origin.position.x/self.map_metadata.resolution  +  point.x/self.map_metadata.resolution
		x = int(round(x))
		y = -self.map_metadata.origin.position.y/self.map_metadata.resolution  +  point.y/self.map_metadata.resolution
		y = self.map_metadata.height - int(round(y)) #images are reversed

		img = np.copy(self.bloated_map)
		cv2.circle(img, (x,y), 1, 150, 2)
		cv2.imshow("Bloated map", img)
		#print self.bloated_map[y][x]
		return self.bloated_map[y][x]

	def centerOfProminentCluster(self, clusters):
		#find biggest cluster
		count = -1
		i_max = -1
		for x in range(len(clusters)):
			temp = len(clusters[x])
			if temp > count:
				i_max = x
				count = temp

		#find its center
		if count > 0:
			#print "Biggest cluster is", i_max, count
			x = 0
			y = 0
			z = 0
			for marker in clusters[i_max]:
				x+=marker.pose.position.x
				y+=marker.pose.position.y
				z+=marker.pose.position.z
			x /= count
			y /= count
			z /= count
			#print "REAL Center is ", x,y,z
			return [x,y,z]

		else:
			return []

	def DBSCAN_markers(self, markers, eps, MinPts):
		tag = 0
		tags = [ -1 for x in range(len(markers)) ] # -1 unvisited, 0 noise, 1+ visited and in cluster
		
		for x in range(len(markers)):
			if tags[x] > 0:
				continue

			tags[x] == 1
			nearbyMarkerIndexes = self.nearbyMarkers(x, eps, markers)
			if len(nearbyMarkerIndexes) < MinPts:
				tags[x] = 0
			else:
				tag += 1
				tags = self.expandCluster(x, nearbyMarkerIndexes, tag, eps, MinPts, markers, tags)

		clusters = [[] for x in range(tag+1)]
		for x in range(len(tags)):
			this_tag = tags[x]
			if this_tag >= 0: #we could ignore the noise
				clusters[this_tag].append(markers[x])

		for x in range(len(clusters)):
			print "Tag:", x, ":", len(clusters[x])

		return clusters[1:]

	def expandCluster(self, P, NeighborPts, tag, eps, MinPts, markers, tags):
		#print "Tagging seed with", tag
		tags[P] = tag    #we tag the current point
		todo = NeighborPts
		done = []
		while todo:
			added = []
			for point in todo:
				if tags[point] < 0:
					tags[point] = 0
					pointContenders = self.nearbyMarkers(point, eps, markers)
					if len(pointContenders) >= MinPts:
						added += pointContenders
				if tags[point] <= 0:
					#print "Tagging link with", tag
					tags[point] = tag
			done+= todo
			todo = added
		return tags

	def nearbyMarkers(self, seed_point, eps, markers):
		seed = markers[seed_point].pose.position
		nearby = [seed_point]

		for x in range(len(markers)):
			p = markers[x].pose.position
			dist = math.sqrt( (seed.x-p.x)*(seed.x-p.x) + (seed.y-p.y)*(seed.y-p.y) + (seed.z-p.z)*(seed.z-p.z) )
			if dist < eps:
				nearby.append(x)

		return nearby #return all markers in eps range of point


	def makeMarker(self, pose, color, id):
		marker = Marker()
		#marker.header.stamp = header.stamp
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = "camera_rgb_optical_frame"
		marker.pose = pose
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		marker.frame_locked = False
		marker.lifetime = rospy.Time(0)
		marker.id = id
		marker.scale = Vector3(0.05, 0.05, 0.05)
		#marker.color = ColorRGBA(1, 1, 1, 1)
		marker.color = ColorRGBA(color[2], color[1], color[0], 1)

		return marker;

	def map_callback(self, grid):
		#std_msgs/Header header
		#nav_msgs/MapMetaData info
		#int8[] data
		self.map_metadata = grid.info
		#time map_load_time
		#float32 resolution
		#uint32 width
		#uint32 height
		#geometry_msgs/Pose origin
		np_arr = np.reshape(grid.data, (grid.info.height, grid.info.width))
		cv_image = np.asarray(np_arr, np.uint8)
		cv_image = cv2.flip(cv_image,0)
		cv_image = (255-cv_image)

		kernel = np.ones((2,2),np.uint8)
		cv_image = cv2.morphologyEx(cv_image,cv2.MORPH_OPEN, kernel, iterations = 2)
		cv_image = cv2.morphologyEx(cv_image,cv2.MORPH_CLOSE, kernel, iterations = 2)
		#cv_image = cv2.erode(cv_image,kernel,iterations = 2)
		retal, cv_image = cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY) #100 is an arbitrary number between 155-255, 155 is the original grayness
		self.bloated_map = cv_image
		cv2.imshow("Bloated map", self.bloated_map)
		#cv2.imwrite("cv_map.png",cv_image)

	def __init__(self):
		#laufat mora rosrun usb_camera usb_cam_node za debuganje preko webCama
		#drugace rabimo pa kinect prizgan

		region_scope = rospy.get_param('~region', 3)
		markers_topic = rospy.get_param('~markers_topic', rospy.resolve_name('%s/markers' % rospy.get_name()))
		image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_color') #kinect
		#image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw') #webcam
		camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/camera_info')		

		map_topic = rospy.get_param('~map_topic', '/map')
		try:
			self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
			rospy.wait_for_message(map_topic, OccupancyGrid, timeout=2)
		except(rospy.ROSException), e:
			print "Map topic is not available, aborting..."
			print "Error message: ", e
		print "Got map"

		print "Waiting for localizer..."
		rospy.wait_for_service('localizer/localize')
		self.localize = rospy.ServiceProxy('localizer/localize', Localize)

		print "Waiting for transform listener..."
		self.listener = TransformListener()
		self.listener.waitForTransform("map", "/camera_rgb_optical_frame", rospy.Time(0), rospy.Duration(5.0))		

		#cv2.namedWindow("Image window", 1) #Control window
		self.bridge = CvBridge()

		print "Creating time synced sub..."
		#self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
		self.image_sub = message_filters.Subscriber(image_topic, Image)
		self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
		self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub], 10)
		self.joined_sub.registerCallback(self.image_callback)
		
		self.markers_pub = rospy.Publisher(markers_topic, MarkerArray, queue_size=10)

		self.calc = rospy.Subscriber('calculate_clusters', Empty, self.calculateCluster)

		self.message_counter = 0

		print rospy.get_name(), "waiting for callbacks..."

# Main function.    
if __name__ == '__main__':

		rospy.init_node('cyllinderdetector')
		try:
			fd = CyllinderDetector()
			rospy.spin()	
		except rospy.ROSInterruptException:
			print "Shutting down"
		
		cv2.destroyAllWindows()

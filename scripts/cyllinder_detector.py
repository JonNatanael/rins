#!/usr/bin/env python
import roslib
#roslib.load_manifest('facedetector')
import rospy, math
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

		markers = MarkerArray()

		for (lower, upper) in self.boundaries: #parse all colors
			
			#print "Checking ", self.color_names[i]
			cyllinderContour = self.findCyllinderContour(hsv, lower, upper, image, camera_model)

			if cyllinderContour.any(): #we got a hit!!!
				ellipse = cv2.fitEllipse(cyllinderContour)
				#print "Center:   " 
				#print ellipse[0]
				
				marker = self.markerFromCoutourEllipse(ellipse, i, image, camera_model)
				if marker:
					#self.all_cyllinders.markers.append(mkr)

					if len(self.markers_by_color[i]) < 5000: #lets not clog the memory and burden the clusterer shall we
						self.markers_by_color[i].append(marker)

				cv2.ellipse(cv_image, ellipse, self.colors[i], 2)
				cv2.drawContours(cv_image, [cyllinderContour], -1, self.colors[i]) 

			i+=1

		if self.message_counter % 100 == 0:

			self.all_cyllinders = MarkerArray()
			#print " "
			#print "Counter:", self.message_counter
			for x in range(len(self.markers_by_color)): #iterate through markers of all colors
				#print "Current color:", self.color_names[x], "markers in bag:", len(self.markers_by_color[x])
				clusters = self.DBSCAN_markers(self.markers_by_color[x], 0.03, 15)

				center = self.centerOfProminentCluster(clusters)
				if center:
					mkr = self.markers_by_color[x][1] #take random properly colored marker
					mkr.pose.position.x = center[0]
					mkr.pose.position.y = center[1]
					mkr.pose.position.z = center[2]
					mkr.scale = Vector3(0.2, 0.2, 0.2)
					self.all_cyllinders.markers.append(mkr)
				#all_cyllinders.markers += self.markers_by_color[x] #add whole color to all_cyllinders

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
		if self.all_cyllinders:
			self.markers_pub.publish(self.all_cyllinders)

		self.message_counter = self.message_counter + 1

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
			ps.header.frame_id = mkr.header.frame_id
			ps.point = mkr.pose.position
			p = self.listener.transformPoint("map", ps)

			mkr.pose.position = ps.point
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
			cArea = 0
			for cnt in contours:
				area = cv2.contourArea(cnt)

				if area < 2000: #minimal size 
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
					if resp and ell[0][1] > 0 and ell[0][0] > 0:
						if resp.pose.position.y == 0 and resp.pose.position.x == 0 and resp.pose.position.y == 0:
							#print "Too close", ell[0]
							continue #response for object too close/too far
						elif resp.pose.position.y > 0 and resp.pose.position.y < 0.2:
							#print "Added", resp.pose.position
							cyllinderContour = cnt
							cArea = area
			
			#if cArea > 0:
			#	print cArea

			return cyllinderContour

	def centerOfProminentCluster(self, clusters):
		#find biggest cluster
		count = -1
		i_max = -1
		for x in range(len(clusters)):
			temp = len(clusters[x])
			if temp > count:
				i_max = x
				count = temp
				print i_max, count

		#find its center
		if count > 0:
			#print "Biggest cluster is", i_max
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
			return None

	def DBSCAN_markers(self, markers, eps, MinPts):
		tag = 1
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
				self.expandCluster(x, nearbyMarkerIndexes, tag, eps, MinPts, markers, tags)

		clusters = [[] for x in range(tag+1)]
		for x in tags:
			this_tag = tags[x]
			if this_tag >= 0: #we could ignore the noise
				clusters[this_tag].append(markers[x])

		for x in range(len(clusters)):
			print "Tag:", x
			print len(clusters[x])

	def expandCluster(self, P, NeighborPts, tag, eps, MinPts, markers, tags):
		tags[P] = tag    #we add the current point to the cluster
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
					tags[point] = tag
			done+= todo
			todo = added

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

	def DBSCAN_markers(self, markers, eps, MinPts):
		tag = 0
		global tags
		self.tags = [ -1 for x in range(len(markers)) ] # -1 unvisited, 0 noise, 1+ visited and in cluster
		
		for x in range(len(markers)):
			if self.tags[x] > 0:
				continue

			self.tags[x] == 0
			nearbyMarkerIndexes = self.nearbyMarkers(x, eps, markers)
			if len(nearbyMarkerIndexes) < MinPts:
				self.tags[x] = 0
			else:
				tag += 1
				self.expandCluster(x, nearbyMarkerIndexes, tag, eps, MinPts, markers)

		clusters = [[] for x in range(tag)]

		for x in range(len(self.tags)):
			this_tag = self.tags[x]
			if x > 0: #ignore the noise
				clusters[this_tag - 1 ].append(markers[x])

		#for x in range(len(clusters)):
			#print "Tag:", x, len(clusters[x])

		return clusters;

	def expandCluster(self, P, NeighborPts, tag, eps, MinPts, markers):
		global tags
		self.tags[P] = tag    #we add the current point to the cluster
		todo = NeighborPts
		done = []
		while todo:
			added = []
			for point in todo:
				if self.tags[point] < 0: #unvisited
					self.tags[point] = 0
					pointContenders = self.nearbyMarkers(point, eps, markers)
					#print len(pointContenders)
					if len(pointContenders) >= MinPts:
						added += pointContenders
				if self.tags[point] == 0: #previously tagged as noise
					self.tags[point] = tag
			done+= todo
			todo = added

	def nearbyMarkers(self, seed_point, eps, markers):
		seed = markers[seed_point].pose.position
		nearby = [seed_point]

		for x in range(len(markers)):
			p = markers[x].pose.position
			dist = math.sqrt( (seed.x-p.x)*(seed.x-p.x) + (seed.y-p.y)*(seed.y-p.y) + (seed.z-p.z)*(seed.z-p.z) )
			if dist < eps:
				nearby.append(x)

		return nearby #return all markers in eps range of point


	def __init__(self):
		#laufat mora rosrun usb_camera usb_cam_node za debuganje preko webCama
		#drugace rabimo pa kinect prizgan

		region_scope = rospy.get_param('~region', 3)
		markers_topic = rospy.get_param('~markers_topic', rospy.resolve_name('%s/markers' % rospy.get_name()))
		image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_color') #kinect
		#image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw') #webcam
		camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/camera_info')		

		print "Waiting for localizer..."
		rospy.wait_for_service('localizer/localize')
		self.localize = rospy.ServiceProxy('localizer/localize', Localize)

		print "Waiting for transform listener..."
		self.listener = TransformListener()
		self.listener.waitForTransform("map", "/camera_rgb_optical_frame", rospy.Time(0), rospy.Duration(5.0))		

		cv2.namedWindow("Image window", 1) #Control window
		self.bridge = CvBridge()

		print "Creating time synced sub..."
		#self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
		self.image_sub = message_filters.Subscriber(image_topic, Image)
		self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
		self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub], 10)
		self.joined_sub.registerCallback(self.image_callback)
		
		self.markers_pub = rospy.Publisher(markers_topic, MarkerArray, queue_size=10)

		self.message_counter = 0

		print "Waiting for callbacks..."

# Main function.    
if __name__ == '__main__':

		rospy.init_node('cyllinderdetector')
		try:
			fd = CyllinderDetector()
			rospy.spin()	
		except rospy.ROSInterruptException:
			print "Shutting down"
		
		cv2.destroyAllWindows()

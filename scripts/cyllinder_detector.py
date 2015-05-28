#!/usr/bin/env python
import roslib
#roslib.load_manifest('facedetector')
import rospy
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA
import sensor_msgs.msg
import message_filters
import cv2
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3

# Node for cyllinder detection.
class CyllinderDetector():

	def image_callback(self, image, camera):
		#we need to bringup minimal.launch
		#we need to bringup 3dsensor.launch

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

		camera_model = PinholeCameraModel()
		camera_model.fromCameraInfo(camera)

		(rows,cols,channels) = cv_image.shape
		if cols > 60 and rows > 60 :
			cv2.circle(cv_image, (50,50), 10, 255)

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
		region_scope = rospy.get_param('~region', 3)
		markers_topic = rospy.get_param('~markers_topic', rospy.resolve_name('%s/markers' % rospy.get_name()))
		image_topic = rospy.get_param('~image_topic', 'camera/rgb/image_color')
		camera_topic = rospy.get_param('~camera_topic', '/camera/camera_info')		

		#rospy.wait_for_service('localizer/localize')
		self.localize = rospy.ServiceProxy('localizer/localize', Localize)


		cv2.namedWindow("Image window", 1) #Control window
		self.bridge = CvBridge()

		self.image_sub = message_filters.Subscriber(image_topic, Image)
		self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
		self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub], 30)
		self.joined_sub.registerCallback(self.image_callback)


		self.localize = rospy.ServiceProxy('localizer/localize', Localize)

		self.markers_pub = rospy.Publisher(markers_topic, MarkerArray)

		self.message_counter = 0

# Main function.    
if __name__ == '__main__':

        rospy.init_node('cyllinderdetector')
        try:
		fd = CyllinderDetector()
		rospy.spin()	
        except rospy.ROSInterruptException: pass

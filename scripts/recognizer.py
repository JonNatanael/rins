#!/usr/bin/env python
import roslib
#roslib.load_manifest('facedetector')
import rospy 
import numpy as np
import sensor_msgs.msg
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from facedetector.msg import Detection
from scipy.io.matlab.mio import loadmat
from scipy.misc import imresize
from scipy.spatial import distance
import glob

class FaceRecognizer():

	def image_callback(self,image, camera, faces):
	#def image_callback(self, image):
		#we need to bringup minimal.launch
		#we need to bringup 3dsensor.launch
		#amcl_demo.launch
		#localizer/localizer
		#rosrun map_server map_server map/map.yaml


		try:
			for i in xrange(0,len(faces.x)):	

				#im = self.bridge.imgmsg_to_cv2(image, "mono8")
				im = self.bridge.imgmsg_to_cv2(faces.image[i], "bgr8")
				#im = np.array(faces.image[i])
				#print faces.image[i]
				#im=cv2.cv.fromarray(im)

				
				#im = cv2.imread(sl)

				im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
				#cv2.imshow("test",im)
				#cv2.waitKey(1)
				im = np.asarray(im)
				#print im
				#print len(im)
				im = imresize(im, (128,128),'bilinear')
				#print im.size
				im = np.reshape(im, (128*128,1),'F') 
				ty = np.dot(np.matrix.transpose(self.U),(im-self.Mu))
				#print ty
				ty = np.dot(np.matrix.transpose(self.V),(ty-self.MM))

				mi = 1e10

				for i in xrange(0,8):
					cr = self.Ms[:,i]
					dist = distance.euclidean(ty,cr)
					if dist<mi:
						mind = i
						mi = dist
				print self.osebe[mind]

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
		#self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub], 10)
		self.joined_sub = message_filters.TimeSynchronizer([self.image_sub, self.camera_sub, self.faces_sub], 20)
		self.joined_sub.registerCallback(self.image_callback)

		# loads matlab matrices
		self.U = loadmat('../recog/u.mat');self.U = self.U['U']
		self.V = loadmat('../recog/v.mat');self.V = self.V['V']
		self.Ms = loadmat('../recog/ms.mat');self.Ms = self.Ms['Ms']
		self.Mu = loadmat('../recog/mu.mat');self.Mu = self.Mu['Mu']
		self.MM = loadmat('../recog/mm.mat');self.MM = self.MM['MM']

		self.osebe = {0:'harry', 1:'ellen',2:'kim',3:'matt',4:'filip',5:'scarlett',6:'tina',7:'prevc'}



# Main function.    
if __name__ == '__main__':

		print "Starting.."
		rospy.init_node('image_saver')
		try:
			fd = FaceRecognizer()
			rospy.spin()	
		except rospy.ROSInterruptException:
			print "Shutting down"
		
		cv2.destroyAllWindows()

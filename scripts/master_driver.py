#!/usr/bin/env python

import roslib
import rospy
import actionlib
from std_msgs.msg import Empty
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
from random import sample
from math import pow, sqrt, sin, cos, atan, pi
from tf import TransformListener, ConnectivityException, Exception, LookupException
from nav_msgs.msg import Odometry

def face_callback(data):
	self.faces = data.poses

def approach_locs_callback(data):
	self.faces_locs = []
	#print data
	for marker in data.markers:
		self.faces_locs.append(marker.pose)

class master_driver():
	#faces = []

	# Goal state return values
	goal_states =[
				'PENDING', 'ACTIVE', 'PREEMPTED', 
				'SUCCEEDED', 'ABORTED', 'REJECTED',
				'PREEMPTING', 'RECALLING', 'RECALLED',
				'LOST'
				]
	loc = [
		Point(0.000, 0.000, 0.000),
		Point(1.03675413132, -0.509295165539, 0.0),
		Point(1.87630188465, -0.331272959709, 0.0),
		Point(2.83038377762, -0.264413654804, 0.0),
		Point(1.65771877766,  0.828545689583, 0.0),
		Point(2.92959189415,  0.880264699459, 0.0),
		Point(3.99440670013,  0.948163211346, 0.0),
		Point(4.89326143265,  0.958462297916, 0.0),
		Point(4.54763841629,  0.054767534136, 0.0),
		Point(5.1187505722,  -0.801208794117, 0.0),
		Point(5.48666477203, -1.92177832127,  0.0),
		Point(5.73681497574, -2.53763198853,  0.0)
		]



	def __init__(self):
		calc = rospy.Publisher('calculate_clusters', Empty)

		rospy.init_node('master_driver', anonymous=True)
		
		rospy.on_shutdown(self.shutdown)
		
		# How long in seconds should the robot pause at each location?
		self.rest_time = rospy.get_param("~rest_time", 2)
		
		# Subscribe to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		
		rospy.loginfo("Waiting for move_base action server...")
		
		# Wait 60 seconds for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(300))
		
		rospy.loginfo("Connected to move base server")
		
		#Subscribe to the facial recognition server

		#sub = rospy.Subscriber('/faces/markers', MarkerArray, face_callback, queue_size=10)
		#sub = rospy.Subscriber('/faces/locations', PoseArray, face_callback, queue_size=10)
		#sub2 = rospy.Subscriber('/faces/approach_points', MarkerArray, approach_locs_callback, queue_size=10)

		# Variables to keep track of success rate, running time,
		# and distance traveled
		n_loc = len(self.loc)
		n_goals = 0
		n_successes = 0
		i = 0
		start_time = rospy.Time.now()
		faces_i = 0
		turning = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
		
		rospy.loginfo("Starting navigation")
		
		# Begin the main loop and run through a sequence of locations
		for point in self.loc:
			self.move(Pose(point, Quaternion(0,0,0,1))) #Quaternion(0.000, 0.000, sin(theta/2), cos(theta/2))
			# rotate in place for detection
			self.rotate(point)

		# send init message to cyllinder_detector and faces
		
		try:			
			calc.publish(Empty())
			print "Published Empty message"
		except e:
			print "Failed publishing empty message"
			print e

		rospy.sleep(self.rest_time)

	def rotate(self, point):
		#Quaternion(0.000, 0.000, sin(theta/2), cos(theta/2))
		for theta in range(0, 720, 120):
			self.move(Pose(point, Quaternion(0, 0, sin(theta), cos(theta))))
			rospy.sleep(2)

		#ALTERNATIVE
		#turningTime = 10
		#twist = Twist()
		#twist.angular.z = pi/turningTime	#radian/s
		#turning.publish(twist) 		#start rotating
		#rospy.sleep(turningTime)	#rotate
		#turning.publish(Twist())	#stop

		# for twCounter in range(1, 10):
		#	 twist = Twist()

		#	 twist.angular.z = 0.8	#radian/s
		#	 turning.publish( twist ) 
		#	 rospy.sleep(0.6) # how long we will turn
			
		#	 turning.publish( Twist() )
		#	 rospy.sleep(2.5)

	def move(self, location):
		self.goal = MoveBaseGoal()
		self.goal.target_pose.pose = location
		self.goal.target_pose.header.frame_id = 'map' 
		self.goal.target_pose.header.stamp = rospy.Time.now()
		
		# Let the user know where the robot is going next
		rospy.loginfo("Going to " + str(location))

		self.move_base.send_goal(self.goal)
			
		# Allow 30 seconds to get there
		finished_within_time = self.move_base.wait_for_result(rospy.Duration(30)) 
			
		# Check for success or failure
		if not finished_within_time:
			self.move_base.cancel_goal()
			rospy.loginfo("Timed out achieving goal")
		else:
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("Goal succeeded!")
				rospy.loginfo("State:" + str(state))
			else:
			  rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))


	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.move_base.cancel_goal()
		rospy.sleep(2)
		#self.cmd_vel_pub.publish(Twist())

def dist(x1,y1,x2,y2):
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

if __name__ == '__main__':
	try:
		faces = []
		faces_locs = []
		master_driver()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AMCL navigation test finished.")

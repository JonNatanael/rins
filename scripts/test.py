#!/usr/bin/env python

import roslib
import rospy
import actionlib
import string
from std_msgs.msg import Empty, String
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
from random import sample
from math import pow, sqrt, sin, cos, atan, pi
from tf import TransformListener, ConnectivityException, Exception, LookupException
from nav_msgs.msg import Odometry


if __name__ == '__main__':
	rospy.init_node('test', anonymous = True)

	calc = rospy.Publisher('/test', Empty, queue_size=10)
	for i in xrange(10):
		calc.publish(Empty())
		rospy.sleep(1)

#!/usr/bin/env python

""" nav_test.py - Version 0.1 2012-01-10

    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    On each round, select a new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
from random import sample
from math import pow, sqrt

def face_callback(data):
    # global faces
    # global faces_i
    #print data
    global faces
    faces = data.markers

class master_driver():
    #faces = []

    def __init__(self):
        rospy.init_node('master_driver', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 2)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        global loc
        loc = []
        
        # loc.append(Pose(Point(0.279, -0.453, 0.000), Quaternion(0.000, 0.000, -0.680, 0.733)))
        # loc.append(Pose(Point(0.279, -0.453, 0.000), Quaternion(0.000, 0.000, 0.947, 0.321)))
        # loc.append(Pose(Point(0.279, -0.453, 0.000), Quaternion(0.000, 0.000, 0.437, 0.899)))

        loc.append(Pose(Point(0.836, -0.338, 0.000), Quaternion(0.000, 0.000, 0.978, 0.210)))
        loc.append(Pose(Point(0.836, -0.338, 0.000), Quaternion(0.000, 0.000, 0.926, -0.378)))

        loc.append(Pose(Point(1.032, -0.553, 0.000), Quaternion(0.000, 0.000, -0.818, 0.576)))

        loc.append(Pose(Point(1.644, -0.241, 0.000), Quaternion(0.000, 0.000, -0.643, 0.765)))
        loc.append(Pose(Point(1.644, -0.241, 0.000), Quaternion(0.000, 0.000, -0.291, 0.957)))

        loc.append(Pose(Point(1.546, 0.554, 0.000), Quaternion(0.000, 0.000, 0.534, 0.845)))
        loc.append(Pose(Point(1.312, 0.609, 0.000), Quaternion(0.000, 0.000, 0.966, -0.258)))

        loc.append(Pose(Point(0.180, 0.843, 0.000), Quaternion(0.000, 0.000, -0.797, 0.604)))

        loc.append(Pose(Point(0.380, 1.197, 0.000), Quaternion(0.000, 0.000, 0.163, 0.987)))
        loc.append(Pose(Point(0.380, 1.197, 0.000), Quaternion(0.000, 0.000, 0.987, -0.163)))

        loc.append(Pose(Point(0.129, 1.760, 0.000), Quaternion(0.000, 0.000, 0.978, 0.208)))
        loc.append(Pose(Point(-0.087, 1.855, 0.000), Quaternion(0.000, 0.000, 0.839, 0.544)))

        loc.append(Pose(Point(1.1, 2.35, 0.000), Quaternion(0.000, 0.000, -0.182, 0.983)))
        loc.append(Pose(Point(1.1, 2.35, 0.000), Quaternion(0.000, 0.000, 0.285, 0.959)))
        loc.append(Pose(Point(1.1, 2.35, 0.000), Quaternion(0.000, 0.000, 0.598, 0.802)))

        
        # Publisher to manually control the robot (e.g. to stop it)
        #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(300))
        
        rospy.loginfo("Connected to move base server")
        
        #Subscribe to the facial recognition server
        sub = rospy.Subscriber('/faces/markers', MarkerArray, face_callback, queue_size=10)


        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_loc = len(loc)
        n_goals = 0
        n_successes = 0
        global i
        i = 0
        start_time = rospy.Time.now()
        global faces_i
        faces_i = 0


        rospy.loginfo("Starting navigation")
        
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():

            if i >= n_loc:
                self.shutdown()
        
            self.move(loc[i])
		            
            #Check if we found any faces and approach them
            while len(faces) > faces_i:
                self.approach()
                faces_i += 1
                self.move(loc[i])

            # Increment the counter
            i += 1

            # Print a summary of faces found and visited
            # rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
            #               str(n_goals) + " = " + 
            #               str(100 * n_successes/n_goals) + "%")

            #move_cmd = Twist()
            #angular_speed = rospy.get_param("~angular_speed", 0.7)
            #move_cmd.angular.z = angular_speed
            #self.cmd_vel.publish(move_cmd)
            #rospy.sleep(4.0)
            
            # stop rotating
            #move_cmd = Twist()
            #self.cmd_vel.publish(move_cmd)
            #rospy.sleep(1.0)
            
            rospy.sleep(self.rest_time)


    def move(self, location):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = location
        self.goal.target_pose.header.frame_id = 'map' 
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        # Let the user know where the robot is going next
        rospy.loginfo("Going to: " + str(location))

        self.move_base.send_goal(self.goal)
            
        # Allow 60 seconds to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            
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
              rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))


    def approach(self):
        #TODO
    	print "approaching face nr.:" + str(faces_i)
        


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        #self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
     
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        faces = []
        master_driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")

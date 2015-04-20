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

import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

class NavTest():
    def __init__(self):
        rospy.init_node('master_driver', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 1)
        
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
        loc = []
        
        loc[0] = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
        loc[1] = Pose(Point(-1.994, 4.382, 0.000), Quaternion(0.000, 0.000, -0.670, 0.743))
        loc[2] = Pose(Point(-3.719, 4.401, 0.000), Quaternion(0.000, 0.000, 0.733, 0.680))
        loc[3] = Pose(Point(0.720, 2.229, 0.000), Quaternion(0.000, 0.000, 0.786, 0.618))
        loc[4] = Pose(Point(1.471, 1.007, 0.000), Quaternion(0.000, 0.000, 0.480, 0.877))
        loc[5] = Pose(Point(-0.861, -0.019, 0.000), Quaternion(0.000, 0.000, 0.892, -0.451))
        
        # Publisher to manually control the robot (e.g. to stop it)
        #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        #initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_loc = len(loc)
        n_goals = 0
        n_successes = 0
        i = 0
        start_time = rospy.Time.now()
        
        # Get the initial pose from the user
        #rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        #self.last_location = Pose()
        #rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        #while initial_pose.header.stamp == "":
        #    rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
        
            # Set up the next goal
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = loc[i]
            self.goal.target_pose.header.frame_id = 'map' 
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Increment the counter
            i+ = 1

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(loc))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 15 seconds to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(15)) 
            
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
            
	    

            # Print a summary of faces found and visited
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            
	    rospy.sleep(self.rest_time)
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")

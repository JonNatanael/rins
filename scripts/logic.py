#!/usr/bin/env python

import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# values should be changed to actual locations, maybe?
obj_clr = {0:'red', 1:'green', 2:'yellow', 3:'blue'}
obj_clr_rev = {'red': 0, 'green':1, 'yellow': 2, 'blue':3}


# temporary answers for testing
osebe = ['prevc', 'scarlett', 'ellen']
clr = ['blue', 'red', 'yellow']
ob = ['teabox', 'cube', 'can']

n_faces = 3
n_cylinders = 4

know = [[0 for x in range(4)] for x in osebe] 
clr = [0 for x in range(4)]

def say(string):
    soundhandle.say(string,voice)
    rospy.sleep(1)

def faces_callback(data):
    faces = data.markers

def cylinders_callback(data):
    cylinders = data.markers

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
          rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

if __name__ == '__main__':


    rospy.init_node('logic', anonymous = True)
    soundhandle = SoundClient()
    rospy.sleep(1)

    global voice
    voice = 'voice_kal_diphone'

    for i in xrange(len(osebe)):
        know[i][0] = osebe[i]

    # TODO subscribe to topics and wait for sufficient lengths of lists
    global faces
    global cylinders

    app_faces = rospy.get_param('~approach_faces', '/approach/faces')
    app_cylinders = rospy.get_param('~approach_cylinders', '/approach_cylinders')
    faces_sub = rospy.Subscriber(app_faces, MarkerArray, self.faces_callback)
    cylinders_sub = rospy.Subscriber(app_cylinders, MarkerArray, self.cylinders_callback)

    while (len(faces)<n_faces and len(cylinders)<n_cylinders):
        rospy.sleep(1)    

    # when all necessary data is acquired

    # fill the first column with names
    for i in xrange(n_faces):
        know[i][0] = faces[i].ns

    # save cylinder colors
    for i in xrange(n_cylinders):
        clr[i] = obj_clr[cylinders[i].id]

    for i in xrange(n_faces):
        person = know[idx][0]
        # move to person's location
        move(faces[i].pose)
        print 'Hi, ' + person.title()
        say('Hi, '+person)
        print 'What color is your hiding place?'
        say('What color is your hiding place?')  
        while True:
            #ans = raw_input("Please input the color:\n")
            ans = clr[idx]
            if ans in obj_clr_rev.keys():
                break
            else:
               print 'Color incorrect'
        # sets the second column to the color name
        know[idx][1] = ans
        # sets the fourth column to the index the cylinder of corresponding color
        know[idx][3] = clr.index(ans)
    print

    idx = 0
    for i in xrange(n_faces):
        person = know[idx][0]
        # gets the correct cylinder for the current person
        cy = cylinders[know[idx][3]]
        # gets the cylinder's color
        cy_clr = know[idx][1]
        print 'Moving to the ' + cy_clr + ' cylinder'
        say('Moving to the ' + cy_clr + ' cylinder')
        # move to appropriate cylinder
        move(cy.pose)
        print 'Please attach object'
        say('Please attach object')
        raw_input('Press enter when done attaching object:\n')
        print 'Moving back to ' + person.title()
        say('Moving back to ' + person
        # move to person's location
        move(faces[i].pose)
        print 'Hello again, ' + person
        say('Hello again, ' + person)
        print 'Which object are you hiding?'
        say('Which object are you hiding?')
        while True:
            #ans = raw_input("Please input the object name:\n")
            ans = ob[idx]
            if ans in ob:
                break
            else:
               print 'Object name incorrect'
        know[idx][2] = ans
        print 'This is your object, right?'
        say('This is your object, right?')
        print 'Please remove object'
        say('Please remove object')
        raw_input('Press enter when done removing object:\n')
        print
        idx+=1
    print

    print 'My final knowledge of the world:'
    for line in know:
        print line[0].title() + ' hid the ' + line[2] + ' under the ' + line[1] + ' cylinder'
        rospy.sleep(1)






    

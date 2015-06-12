#!/usr/bin/env python

import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

goal_states =[
                'PENDING', 'ACTIVE', 'PREEMPTED', 
                'SUCCEEDED', 'ABORTED', 'REJECTED',
                'PREEMPTING', 'RECALLING', 'RECALLED',
                'LOST'
                ]

# values should be changed to actual locations, maybe?
obj_clr = {0:'red', 1:'green', 2:'yellow', 3:'blue'}
obj_clr_rev = {'red': 0, 'green':1, 'yellow': 2, 'blue':3}


faces = []
cylinders = []

# temporary answers for testing
#osebe = ['prevc', 'scarlett', 'ellen']
#clr = ['blue', 'red', 'yellow']
ob = ['teabox', 'cube', 'can']

person_cy = {'scarlett':'red', 'prevts':'blue', 'ellen':'yellow'}
person_ob = {'scarlett':'cube', 'prevts':'teabox', 'ellen':'can'}

n_faces = 3
n_cylinders = 4

know = [[0 for x in range(4)] for x in range(3)] 

def say(string):
    print string
    soundhandle.say(string,voice)
    rospy.sleep(2)

def faces_callback(data):
    #print "FACES"
    #print data.markers
    global faces
    faces = data.markers

def cylinders_callback(data):
    #print "CYLINDERS"
    global cylinders
    cylinders = data.markers

def move(location):
    global move_base
    goal = MoveBaseGoal()
    goal.target_pose.pose = location
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Let the user know where the robot is going next
    rospy.loginfo("Going to " + str(location))

    move_base.send_goal(goal)
        
    # Allow 30 seconds to get there
    finished_within_time = move_base.wait_for_result(rospy.Duration(60)) 
        
    # Check for success or failure
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.loginfo("Timed out achieving goal")
    else:
        state = move_base.get_state()
        print state
        #if state == GoalStatus.SUCCEEDED:
        if state==3:
            rospy.loginfo("Goal succeeded!")
            rospy.loginfo("State:" + str(state))
        else:
            say('I\'m stuck, help!')
            rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

if __name__ == '__main__':
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.init_node('logic', anonymous = True)
    soundhandle = SoundClient()
    rospy.sleep(1)

    global voice
    voice = 'voice_kal_diphone'

    #for i in xrange(3):
    #    know[i][0] = osebe[i]

    # TODO subscribe to topics and wait for sufficient lengths of lists


    app_faces = rospy.get_param('~approach_faces', '/approach/faces')
    app_cylinders = rospy.get_param('~approach_cylinders', '/approach/cylinders')
    faces_sub = rospy.Subscriber(app_faces, MarkerArray, faces_callback)
    cylinders_sub = rospy.Subscriber(app_cylinders, MarkerArray, cylinders_callback)

    #say('prevts')
    #sys.exit()

    #rospy.wait_for_message(app_cylinders, MarkerArray, timeout=20)

    print "waiting for data"
    while (len(faces)<n_faces or len(cylinders)<n_cylinders):
        rospy.wait_for_message(app_faces, MarkerArray, timeout=3000)
        rospy.wait_for_message(app_cylinders, MarkerArray, timeout=3000)  
  

    # when all necessary data is acquired
    print len(faces)
    print len(cylinders)

    # fill the first column with names
    for i in xrange(n_faces):
        know[i][0] = faces[i].ns

    # save cylinder colors
    #if not clr:
    #    for i in xrange(n_cylinders):
    #        clr[i] = obj_clr[cylinders[i].id]

    print know
    #print cylinders

    for i in xrange(n_faces):
        person = know[i][0]
        # move to person's location
        move(faces[i].pose)
        say('Hi, ' + person + ', what color is your hiding place?')
        while True:
            #ans = raw_input("Please input the color:\n")
            #ans = clr[i]
            ans = person_cy[person]
            if ans in obj_clr_rev.keys():
                break
            else:
               print 'Color incorrect'
        # sets the second column to the color name
        know[i][1] = ans
        # sets the fourth column to the index the cylinder of corresponding color
        know[i][3] = obj_clr_rev[ans]
    print
    print know
    #sys.exit()

    for i in xrange(n_faces):
        person = know[i][0]
        # gets the correct cylinder for the current person
        cy = cylinders[know[i][3]]
        # gets the cylinder's color
        cy_clr = know[i][1]
        say('Moving to the ' + cy_clr + ' cylinder')
        # move to appropriate cylinder
        move(cy.pose)
        say('Please attach object')
        raw_input('Press enter when done attaching object:\n')
        say('Moving back to ' + person)
        # move to person's location
        move(faces[i].pose)
        say('Hello again, ' + person + ', which object were you hiding?')
        while True:
            #ans = raw_input("Please input the object name:\n")
            #ans = ob[i]
            ans = person_ob[person]
            if ans in ob:
                break
            else:
               print 'Object name incorrect'
        know[i][2] = ans
        say('This is your object, right?')
        say('Please remove object')
        raw_input('Press enter when done removing object:\n')
        print
    print

    print 'My final knowledge of the world:'
    for line in know:
        say(line[0].title() + ' hid the ' + line[2] + ' under the ' + line[1] + ' cylinder')
        rospy.sleep(1)






    

#!/usr/bin/env python

import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# values should be changed to actual locations, maybe?
obj_clr = {'red': 0, 'green':1, 'yellow': 2, 'blue':3}

# temporary answers for testing
osebe = ['kim', 'harry', 'ellen']
clr = ['red', 'blue', 'green']
ob = ['can', 'tea', 'square']

know = [[0 for x in range(3)] for x in osebe] 

def say(string):
    soundhandle.say(string,voice)
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('say', anonymous = True)
    soundhandle = SoundClient()
    rospy.sleep(1)

    global voice
    voice = 'voice_kal_diphone'

    for i in xrange(len(osebe)):
        know[i][0] = osebe[i]

    idx = 0

    for person in osebe:
        # TODO move to person's location
        #say('Hi, '+person)
        print 'Hi, ' + person.title()
        print 'What color is your hiding place?'
        #say('What color is your hiding place?')  
        while True:
            #ans = raw_input("Please input the color:\n")
            ans = clr[idx]
            if ans in obj_clr.keys():
                break
            else:
               print 'Color not correct'
        know[idx][1] = ans
        

        idx+=1

    print

    idx = 0
    for person in osebe:
        cy = know[idx][1]
        print 'Moving to the ' + cy + ' cylinder'
        #say('Moving to the ' + ans + ' cylinder')
        # TODO move to appropriate cylinder
        print 'Please attach object'
        #say('Please attach object')
        #raw_input('Press enter when done attaching object:\n')
        print 'Moving back to ' + know[idx][0].title()
        #say('Moving back to ' + know[idx][0].title())
        # TODO move to person's location
        print 'Hello again, ' + know[idx][0].title()
        #say('Hello again, ' + know[idx][0])
        print 'Which object are you hiding?'
        #say('Which object are you hiding?')
        while True:
            #ans = raw_input("Please input the color:\n")
            ans = ob[idx]
            if ans in ob:
                break
            else:
               print 'Object name incorrect'
        know[idx][2] = ans
        print 'This is your object, right?'
        #say('This is your object, right?')
        print 'Please remove object'
        #say('Please remove object')
        #raw_input('Press enter when done removing object:\n')
        print
        idx+=1
    print

    print 'My final knowledge of the world:'
    for line in know:
        print line[0].title() + ' hid the ' + line[2] + ' under the ' + line[1] + ' cylinder' 






    

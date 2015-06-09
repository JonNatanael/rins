#!/usr/bin/env python

import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

obj_dict = {'blue':0, 'red':1, 'yellow':2, 'green':3}

osebe = ['kim', 'harry', 'ellen']

clr = ['red', 'blue', 'green']

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
        #move to person's location
        #say('Hi, '+person)
        print 'Hi, ' + person.title()
        print 'What color is your hiding place?'
        #say('What color is your hiding place?')  
        while True:
            #ans = raw_input("Please input the color:\n")
            ans = clr[idx]
            if ans in obj_dict.keys():
                break
            else:
               print 'Color not correct'
        know[idx][1] = ans
        

        idx+=1

    idx = 0
    for person in osebe:
        cy = know[idx][1]
        print 'Moving to the ' + cy + ' cylinder'
        #say('Moving to the ' + ans + ' cylinder')
        #move to appropriate cylinder
        print 'Please attach object'
        #say('Please attach object')
        raw_input('Press enter when done attaching object')
        print 'Moving back to ' + know[idx][0].title()
        #say('Moving back to ' + know[idx][0].title())
        #move to person's location
        print 'Hello again, ' + know[idx][0]
        #say('Hello again, ' + know[idx][0])

        idx+=1
    print know
    #print str(ans)
    #move to specified cylinder
    #say('Moving to the ' + ans + ' cylinder')





    

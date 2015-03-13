#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

#toleranco in pa korake bo treba nastavit trial & error


moveOnX =   [ 0.8,    0,    0,    0, -0.3]
moveOnY =   [ 0  ,    0, 1.35,    0,    0]
rotateOnZ = [ 0  , -2.8,    0, -2.8,    0]
moveOnX =   [ .3, 0,.3,0 ]
moveOnY =   [ 0,0 ,0,0]
rotateOnZ = [ 0,2.1,0,2.1]
step_i = 0



allowedMoveDelta = 0.1
allowedRotDelta = 0.15
moveStep = 0.4
rotStep = 0.4

#pub
#initialState 
update = True
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

def callback(data):
  global update
  global step_i
  global pub
  global initialState
  global moveStep
  global sub
  #super bi bilo met nek user input, da lahko ze od zacetka se pred manual slalomom prizgemo skripto
  #in da se shrani "initial state" kot mi namestimo robota v koordinatni sistem. Tako je neobcutljiv
  #na stanje, v katerem ga pustimo na koncu slaloma
  
  #ocitno se da tudi publishat odometriji...: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
  #(glej odom_pub)

  #probal sem narest zadevo tudi tako, da bi se tudi pri neoptimalni voznji naravnost robotek popravljal

  #z updejti smo vezani na odometry publishe... mogoce ne najbolj pametno?
  #initialState = data.pose.pose
  #print "Step: " + str(step_i)

  if update:
    initialState = data.pose.pose
    update = False

  twist = Twist()

  #ce ni na pravi lokaciji ali X ali Y se bo vozil naprej...
  razl = math.fabs(initialState.position.x - data.pose.pose.position.x)
  print razl
  print step_i
  print math.fabs(math.fabs(moveOnX[step_i])-razl)
  #needsToMoveX = math.fabs(initialState.position.x + moveOnX[step_i] - data.pose.pose.position.x) > allowedMoveDelta
  needsToMoveX = math.fabs(math.fabs(moveOnX[step_i])-razl) > allowedMoveDelta
  #needsToMoveY = math.fabs(initialState.position.y + moveOnY[step_i] - data.pose.pose.position.y) > allowedMoveDelta
  #if  needsToMoveY or needsToMoveX:
  #print razl
  #print math.fabs(moveOnX[step_i]-razl)
  print needsToMoveX
  if needsToMoveX:
    if moveOnX[step_i] < 0:
      twist.linear.x = -moveStep
    else:
      twist.linear.x = moveStep
    pub.publish(twist)
  else: sub.unregister()

  #ne nem kako je s tem kvaternionom, verjetno bo treba delat po nekem modulu namesto absolutnih vrednosti?
  #drugace bo norost ko bo obrnjen 180°
  #rotDir = 0

  #needsToRotate = math.fabs(initialState.orientation.z + rotateOnZ[step_i] - data.pose.pose.orientation.z) > allowedRotDelta
  #needsToRotate = rotateOnZ[step_i]
  #print needsToRotate

  #if  needsToRotate:
  #  turn(-3.1)
    #pub.publish(twist)
   # rotateOnZ[step_i]=0

    #needsToRotate = False
    #if initialState.orientation.z + rotateOnZ[step_i] > data.pose.pose.orientation.z:
    #  rotDir = 1
    #else:
    #  rotDir = -1
    #twist.angular.z = rotDir * rotStep
    #print twist

  #print needsToRotate
  #print "\n\n"
  
  #if (not needsToRotate) and (not needsToMoveY) and (not needsToMoveX):
  #if (not needsToRotate) and (not needsToMoveX):
    #print "a"
    #update = True
    #step_i = step_i + 1
   # if step_i >= len(moveOnX):
      #step_i=step_i-1
     # pub.publish(twist)
    #  rospy.sleep(1)
   #   rospy.signal_shutdown('The path has been completed!')


def moveX():
  print "moving on x"
  sub = rospy.Subscriber('/odom', Odometry, callback, queue_size=10)
  
  #while 1:
  #  if sub:
  #    rospy.sleep(.1)
  #  else:
  #    return

  #rospy.sleep(.1)
  #rospy.spin()
  #sub.unregister()

  #rospy.sleep(1)

def turn(z):
  global pub
  twist = Twist()
  twist.linear.x = 0
  twist.angular.z = z
  #print twist
  print "Turning"
  return twist


def controller():
  #pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
  rospy.init_node('controller', anonymous=True)

  global step_i

  #rospy.Subscriber('/odom', Odometry, callback)
  rospy.sleep(.2)

  for i in range(0,len(moveOnX)):
    step_i = i
    if moveOnX[i]:
      moveX()
    elif rotateOnZ[i]:
      pub.publish(turn(rotateOnZ[i]))
      rospy.sleep(.5)

  #test()

  #rospy.spin()
  #for i in range(1,40):
  # pub.publish(rectangle_movement(i))
  # rospy.sleep(.1)

  #pub.publish(cmd)
  #rospy.sleep(.1)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

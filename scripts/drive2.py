#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(data):
  yrospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def rectangle_movement(step):
  twist = Twist()
  twist.linear.x = 0.4 
  step = step % 20
  if step % 5 == 0:
    twist.linear.x = 0
    twist.angular.z = 2.8
  return twist

def forward(step):
  twist = Twist()
  twist.linear.x = 0.4
  return twist

def turn(z):
  twist = Twist()
  twist.linear.x = 0
  twist.angular.z = z
  return twist

def controller():
  pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
  rospy.init_node('controller', anonymous=True)
  rospy.sleep(.2)

  # forward
  for i in range(1,35):
    pub.publish(forward(i))
    rospy.sleep(.1)

  pub.publish(turn(-3.3))#pozitivno je v levo
  rospy.sleep(1)

  for i in range(1,58):
    pub.publish(forward(i))
    rospy.sleep(.1)

  #pub.publish(turn(-3.1))#pozitivno je v levo
  #rospy.sleep(1)

  #for i in range(1,15):
  #  pub.publish(forward(i))
  #  rospy.sleep(.1)

  ######################## 
  #70cm desno
  #for i in range(1,35):
  #  pub.publish(forward(i))
  #  rospy.sleep(.1)
  #pub.publish(turn(1))
  #rospy.sleep(1)
  #30cm nazaj
  #for i in range(1,15):
  #  pub.publish(forward(i))
  #  rospy.sleep(.1)

  #pub.publish(cmd)
  rospy.sleep(.1)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

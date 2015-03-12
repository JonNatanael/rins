#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def rectangle_movement(step):
  twist = Twist()
  twist.linear.x = 0.4 
  step = step % 20
  if step % 5 == 0:
    twist.linear.x = 0
    twist.angular.z = 2.8
  return twist

def controller():
  pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
  rospy.init_node('controller', anonymous=True)
  rospy.sleep(.2)

  #for i in range(1,40):
  # pub.publish(rectangle_movement(i))
  # rospy.sleep(.1)
  
  twist = Twist()
  twist.linear.x = 0.8 
  pub.publish(twist)
  rospy.sleep(3)

  twist.linear.x = 0.0
  twist.angular.z = -3.1
  pub.publish(twist)
  rospy.sleep(.5)

  twist.linear.x = 1.35
  twist.angular.z = 0
  pub.publish(twist)
  rospy.sleep(4)

  twist.linear.x = 0.0
  twist.angular.z = -3.1
  pub.publish(twist)
  rospy.sleep(.5)

  twist.linear.x = 0.3
  twist.angular.z = 0
  pub.publish(twist)
  rospy.sleep(.3)

  #pub.publish(cmd)
  #rospy.sleep(.1)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

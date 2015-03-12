#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def callback(data):
	print data.pose.pose.position

def rectangle_movement(step):
  twist = Twist()
  twist.linear.x = 0.4 
  step = step % 20
  if step % 10 == 0:
    twist.linear.x = 0
    twist.angular.z = 2.8
  return twist

def controller():
	#pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
  rospy.init_node('controller', anonymous=True)
  rospy.Subscriber("/odom", Odometry, callback)
	#rospy.sleep(.2)

	#for i in range(1,40):
	#	pub.publish(rectangle_movement(i))
	#	rospy.sleep(0.1)
  #while not rospy.is_shutdown():
  #  rospy.sleep(.1)
  rospy.spin()
	#pub.publish(cmd)
	#rospy.sleep(.1)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

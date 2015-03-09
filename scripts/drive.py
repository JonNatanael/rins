#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def controller():
	rospy.init_node('controller', anonymous=True)

	rospy.Subscriber('move_type', String, callback)

	rospy.spin()
    #pub = rospy.Publisher('controller', String, queue_size=10)
    #rospy.init_node('control', anonymous=True)
    #rate = rospy.Rate(2)  # 10hz

    #while not rospy.is_shutdown():
    #    hello_str = "hello world %s" % rospy.get_time()
    #    rospy.loginfo(hello_str)
    #    pub.publish(hello_str)
    #    rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

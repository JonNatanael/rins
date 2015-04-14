import rospy
from geometry_msgs.msg import Twist

def callback(data):
	yrospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def controller():

	location =  Pose(Point(15.322, 11.611, 0.000), Quaternion(0.000, 0.000, 0.629, 0.777))
    self.goal = MoveBaseGoal()
    self.goal.target_pose.pose = location
    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()

    self.move_base.send_goal(self.goal)

    time = self.move_base.wait_for_result(rospy.Duration(300))



if __name__ == '__main__':
    try:
        #controller()
        print "a"
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

moveOnX =   [ 0.8,    0,    0,    0, -0.3]
moveOnY =   [ 0  ,    0, 1.35,    0,    0]
rotateOnZ = [ 0  , -2.8,    0, -2.8,    0]
step_i = 0

#toleranco in pa korake bo treba nastavit trial & error
allowedMoveDelta = 0.5
allowedRotDelta = 0.15
moveStep = 0.4
rotStep = 0.1

pub
initialState 
update = True

def callback(data):

  #super bi bilo met nek user input, da lahko ze od zacetka se pred manual slalomom prizgemo skripto
  #in da se shrani "initial state" kot mi namestimo robota v koordinatni sistem. Tako je neobcutljiv
  #na stanje, v katerem ga pustimo na koncu slaloma
  
  #ocitno se da tudi publishat odometriji...: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
  #(glej odom_pub)

  #probal sem narest zadevo tudi tako, da bi se tudi pri neoptimalni voznji naravnost robotek popravljal

  #z updejti smo vezani na odometry publishe... mogoce ne najbolj pametno?

  if update:
    initialState = data.pose.pose
    update = false

  twist = Twist()

  #ce ni na pravi lokaciji ali X ali Y se bo vozil naprej...
  needsToMoveX = math.fabs(initialState.position.x + moveOnX[step_i] - data.pose.pose.position.x) > allowedMoveDelta
  needsToMoveY = math.fabs(initialState.position.y + moveOnY[step_i] - data.pose.pose.position.y) > allowedMoveDelta
  if  needsToMoveY or needsToMoveX:
    twist.linear.x = moveStep

  #ne nem kako je s tem kvaternionom, verjetno bo treba delat po nekem modulu namesto absolutnih vrednosti?
  #drugace bo norost ko bo obrnjen 180°
  rotDir = 0
  needsToRotate = math.fabs(initialState.orientation.z + rotateOnZ[step_i] - data.pose.pose.orientation.z) > allowedRotDelta
  if  needsToRotate:
    if initialState.orientation.z + rotateOnZ[step_i] > data.pose.pose.orientation.z:
      rotDir = 1
    else:
      rotDir = -1
    twist.angular.z = rotDir * rotStep
  
  if !needsToRotate and !needsToMoveY and !needsToMoveX
    update = true
    step_i++
    if step_i >= len(moveOnX) 
      rospy.signal_shutdown(“The path has been completed!”)

  pub.publish(twist)


def controller():
  pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
  rospy.init_node('controller', anonymous=True)

  rospy.Subscriber('/odom', Odometry, callback)
  rospy.sleep(.2)

  rospy.spin()
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

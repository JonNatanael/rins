#!/usr/bin/env python
import roslib
#roslib.load_manifest('faces')
import rospy
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA,Header
import sensor_msgs.msg
import message_filters
from facedetector.msg import Detection
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3, PoseArray, Pose, Quaternion, PointStamped
from math import sin, cos, sqrt
from tf import TransformListener, TransformerROS
from tf.transformations import euler_from_quaternion

# Node for face detection.
class FaceMapper():

    def faces_callback(self, faces, camera):
        #print "faces"

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera)

        n = len(faces.x) ##faces ... .x ?!

        markers = MarkerArray()

        for i in xrange(0, n):
            u = faces.x[i] + faces.width[i] / 2
            v = faces.y[i] + faces.height[i] / 2
            # TODO limit detections on y coordinate
            #print u, v
            point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
                 ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)
            
            #print point
            resp = self.localize(faces.header, point, 3)
            if resp:
                #print resp
                marker = Marker()
                marker.header.stamp = faces.header.stamp
                marker.header.frame_id = faces.header.frame_id
                marker.pose = resp.pose
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.frame_locked = False
                marker.lifetime = rospy.Time(0)
                marker.id = len(self.faces_list)
                marker.scale = Vector3(0.1, 0.1, 0.1)
                marker.color = ColorRGBA(1, 0, 0, 1)
                #print marker

                # TODO transform current face position to map coordinates
                #listener = TransformListener()
                #rospy.sleep(4.0)
                
                trans = (0,0,0)
                rot = (0,0,1,0)
                p = PointStamped()

                try:
                    listener = TransformListener()
                    listener.waitForTransform("/map", "/camera_rgb_optical_frame", rospy.Time(0), rospy.Duration(2.0))
                    (trans, rot) = listener.lookupTransform('/map', '/camera_rgb_optical_frame', faces.header.stamp+rospy.Duration(4.0))
                    #(trans, rot) = listener.lookupTransform('/map', '/camera_rgb_optical_frame', faces.header.stamp)
                    ps = PointStamped()
                    ps.header.stamp = rospy.Time()
                    ps.header.frame_id = faces.header.frame_id
                    ps.point = marker.pose.position
                    #t = TransformerROS()
                    p = listener.transformPoint('/map', ps)
                    print p
                    #print trans,rot
                except Exception as ex:
                    print "e"
                    print ex

                #x1 = marker.pose.position.x+trans[0]*sin(rot[2])
                #y1 = marker.pose.position.z+trans[1]*cos(rot[3])
                #x1 = trans[0]+p.point.x
                x1 = p.point.x
                y1 = p.point.y
                #y1 = trans[1]+p.point.y
                print x1,y1,trans[2]


                # TODO compare these coordinates to all previously detected faces

                if abs(resp.pose.position.y) < self.height_limit:
                    if len(self.faces_list)>0:
                        in_range = False
                        for j in xrange(0,len(self.faces_list)):
                            #if self.dist(self.faces_list[j].pose.position.x,self.faces_list[j].pose.position.y,resp.pose.position.x,resp.pose.position.y) < self.dist_limit:
                            if self.dist(self.faces_locs.poses[j].position.x, self.faces_locs.poses[j].position.y, x1, y1) < self.dist_limit:
                                in_range = False#True
                        if not in_range:	
                            if marker.pose.position.z > 0:
                              self.faces_list.append(marker)
                              pose = Pose(Point(x1, y1, 0.66), Quaternion(0, 0, 1, 0))
                              self.faces_locs.poses.append(pose)
                              self.app_points.markers.append(self.createMarker(pose, faces.header))
                    else:
                        if marker.pose.position.z > 0:
                            self.faces_list.append(marker)
                            pose = Pose(Point(x1, y1, 0.66), Quaternion(0, 0, 1, 0))
                            self.faces_locs.poses.append(pose)
                            self.app_points.markers.append(self.createMarker(pose, faces.header))

        #add all previously detected faces
        for face in self.faces_list:
            markers.markers.append(face)
            #print face

        print len(self.faces_list)
        #print markers

        self.markers_pub.publish(markers) #to so markerji ki jih pustimo rvizu transformirat
        self.locations_pub.publish(self.faces_locs) #to so VSI markerji, samo da jih mi transformiramo na mapo?
        #self.approach_point_pub.publish(makeClusters(self.app_points))
        self.approach_point_pub.publish(self.app_points) #kako je to drugace od faces_locs? mogoce bi blo dobro mojo neumnost preimenovat...

        self.message_counter = self.message_counter + 1


    def dist(self,x1,y1,x2,y2):
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

    def createMarker(self,pose,header):
    	mrkr = Marker()
        mrkr.header.stamp = header.stamp
        #mrkr.header.frame_id = header.frame_id
        mrkr.header.frame_id = 'map'
        mrkr.pose = pose
        mrkr.type = Marker.CUBE
        mrkr.action = Marker.ADD
        mrkr.frame_locked = False
        mrkr.lifetime = rospy.Time(0)
        mrkr.id = len(self.faces_list)
        mrkr.scale = Vector3(0.1, 0.1, 0.1)
        mrkr.color = ColorRGBA(0, 1, 0, 1)
        return mrkr

    def __init__(self):
        region_scope = rospy.get_param('~region', 3)
        markers_topic = rospy.get_param('~markers_topic', rospy.resolve_name('%s/markers' % rospy.get_name()))
        locations_topic = rospy.get_param('~locations_topic', rospy.resolve_name('%s/locations' % rospy.get_name()))
        approach_point_topic = rospy.get_param('~approach_point_pub', rospy.resolve_name('%s/approach_points' % rospy.get_name()))

        faces_topic = rospy.get_param('~faces_topic', '/facedetector/faces')
        camera_topic = rospy.get_param('~camera_topic', '/camera/camera_info')      

        rospy.wait_for_service('localizer/localize')

        self.faces_sub = message_filters.Subscriber(faces_topic, Detection)
        self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
        self.joined_sub = message_filters.TimeSynchronizer([self.faces_sub, self.camera_sub], 30)
        self.joined_sub.registerCallback(self.faces_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)

        self.markers_pub = rospy.Publisher(markers_topic, MarkerArray)
        self.markers_pub.publish([])

        self.locations_pub = rospy.Publisher(locations_topic, PoseArray)
        #self.locations_pub.publish(self.)

        self.approach_point_pub = rospy.Publisher(approach_point_topic, MarkerArray)
        self.approach_point_pub.publish([])

        self.message_counter = 0

        self.app_points = MarkerArray()

        self.faces_list = []
        self.faces_locs = PoseArray(Header(),[])
        self.dist_limit = 0.5
        self.height_limit = 0.2

        # init transform listener
        #self.listener = TransformListener()
        #self.listener.waitForTransform("/map", "/odom", rospy.Time(), rospy.Duration(10.0))
        #(trans,rot)=self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
        #print trans,rot
        #rospy.sleep(2.0)



# Main function.    
if __name__ == '__main__':

    rospy.init_node('facemapper')
    try:
        print "here"
        #listener = TransformListener()
        #listener.waitForTransform("/map", "/camera_rgb_optical_frame", rospy.Time(), rospy.Duration(10.0))
    	fd = FaceMapper()
        rospy.spin()    
    except rospy.ROSInterruptException: pass

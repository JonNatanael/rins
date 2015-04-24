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
from math import sin, cos, sqrt, atan
from tf import TransformListener, TransformerROS
from tf.transformations import euler_from_quaternion
#from clusters import makeFaceClusters

# Node for face detection.
class FaceMapper():

    def faces_callback(self, faces, camera):
        #print "faces"

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera)

        n = len(faces.x)

        markers = MarkerArray()
        clusteringResults = PoseArray(Header(),[])

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
                    (trans, rot) = listener.lookupTransform('/map', '/camera_rgb_optical_frame', rospy.Time(0))
                    #(trans, rot) = listener.lookupTransform('/map', '/camera_rgb_optical_frame', faces.header.stamp)
                    ps = PointStamped()
                    ps.header.stamp = rospy.Time()
                    ps.header.frame_id = faces.header.frame_id
                    ps.point = marker.pose.position
                    #t = TransformerROS()
                    p = listener.transformPoint('/map', ps)
                    #print p
                    #print trans,rot

                    #x1 = marker.pose.position.x+trans[0]*sin(rot[2])
                    #y1 = marker.pose.position.z+trans[1]*cos(rot[3])
                    #x1 = trans[0]+p.point.x
                    x1 = p.point.x
                    y1 = p.point.y
                    #y1 = trans[1]+p.point.y
                    #print x1,y1,trans[2]


                    # TODO compare these coordinates to all previously detected faces

                    if abs(resp.pose.position.y) < self.height_limit:
                        if len(self.faces_list)>0:
                            in_range = False
                            for j in xrange(0,len(self.faces_list)):
                                #if dist(self.faces_list[j].pose.position.x,self.faces_list[j].pose.position.y,resp.pose.position.x,resp.pose.position.y) < self.dist_limit:
                                if dist(self.faces_locs.poses[j].position.x, self.faces_locs.poses[j].position.y, x1, y1) < self.dist_limit:
                                    in_range = True
                            if not in_range:	
                                if marker.pose.position.z > 0:
                                  self.faces_list.append(marker)
                                  pose = Pose(Point(x1, y1, 0.66), Quaternion(0, 0, 1, 0))
                                  self.faces_locs.poses.append(pose)
                                  #self.app_points.markers.append(self.createMarker(pose, faces.header))
                                  pose = self.calculateApproach(self.faces_locs.poses[len(self.faces_locs.poses)-1],x1,y1)
                                  if pose is not None:
                                    self.app_points.markers.append(self.createMarker(pose, faces.header))
                        else:
                            if marker.pose.position.z > 0:
                                self.faces_list.append(marker)
                                pose = Pose(Point(x1, y1, 0.66), Quaternion(0, 0, 1, 0))
                                self.faces_locs.poses.append(pose)
                                pose = self.calculateApproach(self.faces_locs.poses[len(self.faces_locs.poses)-1],x1,y1)
                                if pose is not None:
                                    self.app_points.markers.append(self.createMarker(pose, faces.header))

                    # CLUSTERING
                    if abs(resp.pose.position.y) < self.height_limit:
                        pose = Pose(Point(x1, y1, 0.66), Quaternion(0, 0, 1, 0))
                        self.allDetected.poses.append(pose)
                        clusteringResults = PoseArray(Header(),[])
                        clusteringResults.header.frame_id = 'map'
                        for (xCluster, yCluster, unused1, unused2) in makeFaceClusters(self, self.allDetected):
                            clusteringResults.poses.append(Pose(xCluster, yCluster, 0.50), Quaternion(0, 0, 1, 0))

                except Exception as ex:
                    print "e"
                    print ex

        #add all previously detected faces
        for face in self.faces_list:
            markers.markers.append(face)
            #print face

        print len(self.faces_list)
        #print markers

        self.markers_pub.publish(markers)
        self.locations_pub.publish(clusteringResults)

        self.approach_point_pub.publish(self.app_points)

        self.message_counter = self.message_counter + 1

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

    def calculateApproach(self,faceIndex,x1,y1):
        try:
            listener = TransformListener()
            listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
            #if listener.canTransform("/map", "/base_link",rospy.Time()):
            #time = listener.getLatestCommonTime("/map", "/base_link")
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())

            dist = 0.35 # distance from face
            x2 = trans[0]
            y2 = trans[1]
            if (abs((x2 - x1)**2 + (y2 - y1)**2) < dist):
                x3 = x2
                y3 = y2
            else:
                m = abs((y2-y1)/(x2-x1)) # slope
                offsetX = dist * 1/sqrt(1 + m**2)
                offsetY = dist * m/sqrt(1 + m**2)
                if (x2 > x1):
                    x3 = x1 + offsetX
                else:
                    x3 = x1 - offsetX
                if (y2 > y1):
                    y3 = y1 + offsetY
                else:
                    y3 = y1 - offsetY

            m = (trans[1]-y1)/(trans[0]-x1) # slope
            theta = atan(m)
            return Pose(Point(x3, y3, 0.000), Quaternion(0.000, 0.000, sin(theta/2), cos(theta/2)))
        except Exception as ex:
            print "fail"
            print ex
        return None

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
        self.locations_pub.publish(Header(), [])

        self.approach_point_pub = rospy.Publisher(approach_point_topic, MarkerArray)
        self.approach_point_pub.publish([])

        self.message_counter = 0

        self.app_points = MarkerArray()

        self.faces_list = []
        self.faces_locs = PoseArray(Header(),[])
        self.allDetected = PoseArray(Header(),[])
        self.dist_limit = 1.3
        self.height_limit = 0.2

        # init transform listener
        #self.listener = TransformListener()
        #self.listener.waitForTransform("/map", "/odom", rospy.Time(), rospy.Duration(10.0))
        #(trans,rot)=self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
        #print trans,rot
        #rospy.sleep(2.0)

def dist(x1,y1,x2,y2):
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

def makeFaceClusters(self, hits):
# spremenjena verzija, bo mogoce boljsi preformance
# zdaj prejmo PoseArray, ne MarkerArray

    num_closest = 15 #how many must be in the desired range to be consisered a cluster
    spread = 0.35 #how small must the cluster be
    threshold = 0.5 #how close can clusters be one another

    #find all contenders. raw is a list of tuples: cluster(x_center, y_center, max_distance_from_center)
    raw = []
    for marker in hits.poses:

        #sorted_by_dist = sorted(hits, key=self.dist(hits.))
        points_in_range = 0
        max_dist = 0
        points_cluster_xy = []

        for contender in hits.poses: #O(n^2), yay!

            if contender == marker: #if it's our center ignore it
                continue

            #get the distance from the center point
            dst = dist(marker.position.x, marker.position.y, contender.position.x, contender.position.y)
            if dst < spread: #if it's inside the spread add to the cluster size counter 
                points_in_range += 1
                points_cluster_xy.append((contender.position.x, contender.position.y))

        if points_in_range > num_closest: #if we had enough nearby points to consider this a cluster
            raw.append((marker.position.x, marker.position.y, points_in_range, points_cluster_xy)) #add our initial center and number of points in cloud

    #we should now have a list of all cluster centers stored in raw[]
    #we need to define our logic for what the best clusters are. In this instance I assume the tightest one as in lowest max_dist
    sorted_raw = sorted(raw, key=lambda tup: tup[2])

    clusters = []
    if len(sorted_raw) > 0:
        clusters.append(sorted_raw[0]) #the tightest one if we have one can automatically be added

    for i in xrange(1, len(sorted_raw)):

        #for each contending cluster check if there isn't one (tighter, better) added to the list withun its range 
        below_thresh = False
        for appoved_cluster in clusters:
            dst = dist(appoved_cluster[0], appoved_cluster[1], sorted_raw[i][0], sorted_raw[i][1])

            if dst < threshold: 
                below_thresh=True

        if not below_thresh:
            clusters.append(sorted_raw[i])

    return clusters #returns a list of tuples: tup(cluster_center.x, cluster_center.y, cluster_sperad/2)




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

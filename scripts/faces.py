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
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, PointStamped
from math import sin, cos, sqrt, atan
from tf import TransformListener, TransformerROS
from tf.transformations import euler_from_quaternion
from rins.msg import StampedString

# Node for face detection.
class FaceMapper():

    def faces_callback(self, faces, camera, recog):
        #print recog

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera)

        n = len(faces.x)

        clusteringResults = MarkerArray()

        for i in xrange(0, n):
            u = faces.x[i] + faces.width[i] / 2
            v = faces.y[i] + faces.height[i] / 2
            point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
                 ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)            
            
            resp = self.localize(faces.header, point, 3)
            if resp:
                marker = self.createMarker(resp.pose, faces.header)
                marker.header.frame_id = faces.header.frame_id
                
                trans = (0,0,0)
                rot = (0,0,1,0)
                p = PointStamped()

                try:                    
                    (trans, rot) = self.listener.lookupTransform('/map', '/camera_rgb_optical_frame', rospy.Time(0))
                    ps = PointStamped()
                    ps.header.stamp = rospy.Time()
                    ps.header.frame_id = faces.header.frame_id
                    ps.point = marker.pose.position
                    p = self.listener.transformPoint('/map', ps)
                    marker.pose.position.x = p.point.x
                    marker.pose.position.y = p.point.y
                    marker.ns = recog.s[i]

                    if abs(resp.pose.position.y) < self.height_limit:
                        if marker.pose.position.z > 0:
                            self.faces_list.markers.append(marker)
                            #self.faces_locs.markers.append(marker)
                
                except Exception as ex:
                    print "exception"
                    print ex

        #clusteringResults.header.frame_id = 'map'
        for (xCluster, yCluster, name) in self.makeFaceClusters(self.faces_list):
            h = Header()
            h.frame_id = 'map'
            mkr = self.createMarker(Pose(Point(xCluster, yCluster, 0.50),Quaternion(0,0,1,0)),h)
            mkr.ns = name
            mkr.id = self.osebe_rev[name]
            clusteringResults.markers.append(mkr)

        self.markers_pub.publish(self.faces_list)
        self.locations_pub.publish(clusteringResults)


    def createMarker(self,pose,header):
    	mrkr = Marker()
        mrkr.header.stamp = header.stamp
        mrkr.header.frame_id = 'map'
        mrkr.pose = pose
        mrkr.type = Marker.CUBE
        mrkr.action = Marker.ADD
        mrkr.frame_locked = False
        mrkr.lifetime = rospy.Time(0)
        mrkr.id = len(self.faces_list.markers)
        mrkr.scale = Vector3(0.1, 0.1, 0.1)
        mrkr.color = ColorRGBA(0, 1, 0, 1)
        return mrkr
    


    def dist(self,x1,y1,x2,y2):
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

    def makeFaceClusters(self, hits):
        # spremenjena verzija, bo mogoce boljsi preformance
        # zdaj prejmo PoseArray, ne MarkerArray
        # zdaj spet MarkerArray

        num_closest = 5 #how many must be in the desired range to be consisered a cluster
        spread = 0.5 #how small must the cluster be
        threshold = 1 #how close can clusters be one another

        #find all contenders. raw is a list of tuples: cluster(x_center, y_center, max_distance_from_center)
        raw = []
        for marker in hits.markers:
            print marker.ns

            #sorted_by_dist = sorted(hits, key=self.dist(hits.))
            points_in_range = 0
            max_dist = 0
            points_cluster_xy = []
            recog_cnt = [0 for x in range(8)]

            for contender in hits.markers: #O(n^2), yay!

                if contender == marker: #if it's our center ignore it
                    continue

                #get the distance from the center point
                dst = self.dist(marker.pose.position.x, marker.pose.position.y, contender.pose.position.x, contender.pose.position.y)
                if dst < spread: #if it's inside the spread add to the cluster size counter 
                    points_in_range += 1
                    recog_cnt[self.osebe_rev[contender.ns]] +=1
                    points_cluster_xy.append((contender.pose.position.x, contender.pose.position.y))

            #print recog_cnt.index(max(recog_cnt))
            name = self.osebe[recog_cnt.index(max(recog_cnt))]

            if points_in_range > num_closest: #if we had enough nearby points to consider this a cluster
                raw.append((marker.pose.position.x, marker.pose.position.y, points_in_range, points_cluster_xy, name)) #add our initial center and number of points in cloud

        #we should now have a list of all cluster centers stored in raw[]
        #we need to define our logic for what the best clusters are. In this instance I assume the tightest one as in lowest max_dist
        sorted_raw = sorted(raw, key=lambda tup: tup[2])

        clusters = []
        if len(sorted_raw) > 0:
            adjustedFace = self.clusterCenter(sorted_raw[0][3],sorted_raw[0][4])
            clusters.append(adjustedFace) #the tightest one if we have one can automatically be added

        for i in xrange(1, len(sorted_raw)):        
            adjustedFace = self.clusterCenter(sorted_raw[i][3],sorted_raw[i][4])

            #for each contending cluster check if there isn't one (tighter, better) added to the list withun its range 
            below_thresh = False
            for appoved_cluster in clusters:
                dst = self.dist(appoved_cluster[0], appoved_cluster[1], adjustedFace[0], adjustedFace[1])

                if dst < threshold: 
                    below_thresh=True

            if not below_thresh:
                clusters.append(adjustedFace)

            if len(clusters) == 3:
                break

        return clusters #returns a list of tuples: tup(cluster_center.x, cluster_center.y)

    def clusterCenter(self, points_cluster_xy, name):
        centerX = 0.0
        centerY = 0.0
        for (newX, newY) in points_cluster_xy:
            centerX += newX
            centerY += newY
        centerX /= len(points_cluster_xy)
        centerY /= len(points_cluster_xy)
    
        return (centerX, centerY, name)

    def __init__(self):
        self.listener = TransformListener()
        self.listener.waitForTransform("/map", "/camera_rgb_optical_frame", rospy.Time(0), rospy.Duration(2.0))

        region_scope = rospy.get_param('~region', 3)
        markers_topic = rospy.get_param('~markers_topic', rospy.resolve_name('%s/markers' % rospy.get_name()))
        locations_topic = rospy.get_param('~locations_topic', rospy.resolve_name('%s/locations' % rospy.get_name()))
        approach_point_topic = rospy.get_param('~approach_point_pub', rospy.resolve_name('%s/approach_points' % rospy.get_name()))

        faces_topic = rospy.get_param('~faces_topic', '/facedetector/faces')
        camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/camera_info')   
        recog_topic = rospy.get_param('~recog_topic', '/recognizer')

        rospy.wait_for_service('localizer/localize')

        self.faces_sub = message_filters.Subscriber(faces_topic, Detection)
        self.camera_sub = message_filters.Subscriber(camera_topic, CameraInfo)
        self.recog_sub = message_filters.Subscriber(recog_topic, StampedString)
        self.joined_sub = message_filters.TimeSynchronizer([self.faces_sub, self.camera_sub, self.recog_sub], 30)
        self.joined_sub.registerCallback(self.faces_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)

        self.markers_pub = rospy.Publisher(markers_topic, MarkerArray, queue_size=10)
        self.markers_pub.publish([])

        self.locations_pub = rospy.Publisher(locations_topic, MarkerArray, queue_size=10)
        self.locations_pub.publish([])

        self.message_counter = 0

        self.faces_list = MarkerArray()
        self.dist_limit = .5
        self.height_limit = 0.2
        self.osebe = {0:'harry', 1:'ellen',2:'kim',3:'matt',4:'filip',5:'scarlett',6:'tina',7:'prevc'}
        self.osebe_rev = {'harry':0, 'ellen':1,'kim':2,'matt':3,'filip':4,'scarlett':5,'tina':6,'prevc':7}

        print "b"

# Main function.    
if __name__ == '__main__':

    rospy.init_node('facemapper')
    try:
    	fd = FaceMapper()
        rospy.spin()    
    except rospy.ROSInterruptException: pass

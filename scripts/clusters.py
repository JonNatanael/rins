
def makeClusters(self, hits):

    num_closest = 15 #how many must be in the desired range to be consisered a cluster
    spread = 0.35 #how small must the cluster be
    threshold = 0.5 #how close can clusters be one another

    #find all contenders. raw is a list of tuples: cluster(x_center, y_center, max_distance_from_center)
    raw = []
    for marker in hits.markers:

        position = marker.pose
        #sorted_by_dist = sorted(hits, key=self.dist(hits.))
        points_in_range = 0
        max_dist = 0

        for contender in hits.markers: #O(n^2), yay!

            if contender == marker: #if it's our center ignore it
                continue

            #get the distance from the center point
            dst = dist(marker.pose.point.x, marker.pose.point.y, contender.pose.point.x, contender.pose.point.y)
            if dst < spread: #if it's inside the spread add to the cluster size counter 
                points_in_range += 1
                if dst > max_dist: #adjust the max dist
                    max_dist = dst

        if points_in_range > num_closest: #if we had enough nearby points to consider this a cluster
            raw.append((marker.pose.point.x, marker.pose.point.y, max_dist)) #add our initial center with the furthest point inside the cloud info intact

    #we should now have a list of all cluster centers stored in raw[]
    #we need to define our logic for what the best clusters are. In this instance I assume the tightest one as in lowest max_dist
    sorted_raw = sorted(raw, key=lambda tup: tup[2])

    clusters = []
    if len(sorted_raw) > 0:
        clusters.append(sorted_raw[0]) #the tightest one if we have one can automatically be added

    for i in xrange(1, len(sorted_raw)):

        #for each contending cluster check if there isn't one (tighter, better) added to the list withun its range 
        below_thresh = false
        for appoved_cluster in clusters:
            dst = dist(appoved_cluster[0], appoved_cluster[1], sorted_raw[i][0], sorted_raw[i][1])

            if dst < threshold: 
                below_thresh=True

        if not below_thresh:
            clusters.append(sorted_raw[i])

    return clusters #returns a list of tuples: tup(cluster_center.x, cluster_center.y, cluster_sperad/2)



#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from std_msgs.msg import Int32
import numpy as np
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 30 #200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)   
        #rospy.Subscriber('/obstacle_waypoint', , obstacle_cb)  # i don't see '/obstacle_waypoint' topic... 
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_wps = None  # store /base_waypoints
        self.waypoints_2d = None #store base_waypoints in 2D list form
        self.waypoint_tree = None #store base_waypoints in KD Tree form (for quick lookup)
        self.pose = None  #store /current_pose
        self.velocity = None #store /current_velocity
        self.stopline_wp_idx = -1 #for stop line waypoint from traffic light detection results
        self.loop()
        #rospy.spin()

    def loop(self): #loop at 50 Hz for publishing final_waypoints
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_wps: #only process if there is data
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        closest_coord = self.waypoints_2d[closest_idx]
        # tell if cloest_idx is in front or behind the vehicle
        prev_coord = self.waypoints_2d[(closest_idx-1+len(self.waypoints_2d)) % len(self.waypoints_2d)] # add len(self.waypoints_2d) in case the closest_idx happens to be 0
        val = np.dot(np.array(closest_coord)-np.array(prev_coord), np.array(closest_coord)-np.array([x, y])) 
        if val>=0:
            return closest_idx
        else:
            return (closest_idx+1) % len(self.waypoints_2d)
       
    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_wps.header                

        # craft base_waypoints
        base_waypoints = self.base_wps.waypoints[closest_idx:(closest_idx+LOOKAHEAD_WPS)]
        if len(base_waypoints)<LOOKAHEAD_WPS:
            base_waypoints.extend(self.base_wps.waypoints[:(LOOKAHEAD_WPS-len(base_waypoints))])
        
        # select waypoints for lane message, depending on traffic light detection results
        ##TH_DECEL_WPS = 50  #decelerate if diff in waypoints between light and car position is less than this threshold
        
        if self.stopline_wp_idx == -1 or ((self.stopline_wp_idx-closest_idx+len(self.waypoints_2d)) % len(self.waypoints_2d) >=LOOKAHEAD_WPS):
            lane.waypoints = base_waypoints  #use normal waypoints, cruise speed ~25mph (11m/s)
        else:
            rospy.logwarn('!!!need to decelerate, current speed = %d m/s', self.velocity.twist.linear.x)
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        self.final_waypoints_pub.publish(lane)

    def decelerate_waypoints(self, waypoints, closest_idx):
        # modify waypoints if we need to decelerate (due to traffic light)
        temp = []
        MAX_DECEL = 0.5
        stop_idx = max(((self.stopline_wp_idx-closest_idx+len(self.waypoints_2d)) % len(self.waypoints_2d))-3, 0) #three waypoints back from line (car center is about 2 waypoints from front)
        dist = self.distance(waypoints, 0, stop_idx)  #distance to traffic light stop line
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            #p.pose = wp.pose #not necessary, remove to reduce cpu load
            if i>0:            
                dist = dist - self.distance(waypoints, i-1, i)
            if dist<0:
                dist = 0
            vel = math.sqrt(2*MAX_DECEL*dist)  #decrease velocity, will be 0 when dist=0
            if vel<1.:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def pose_cb(self, msg): #respond to /current_pose topic
        # TODO: Implement
        self.pose = msg

    def velocity_cb(self, msg): #respond to /current_velocity topic
        self.velocity = msg

    def waypoints_cb(self, waypoints):  #respond to /base_waypoints topic
        # TODO: Implement
        self.base_wps = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)        
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement (DONE)
        self.stopline_wp_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

import datetime

from scipy.spatial import KDTree
import numpy as np

STATE_COUNT_THRESHOLD = 2 #3, change to 2 for faster respone, the detection seems to be stable

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.closest_light = None # to store the next traffic light information
        ## additional variables
        self.waypoints_2d = None #store base_waypoints in 2D list form
        self.waypoint_tree = None #store base_waypoints in KD Tree form (for quick lookup)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.imageCounter = 0 #counter for selectively processing images, to mitigate latency in simulator

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.imageCounter = (self.imageCounter+1) % 5  #only process image every 5 frames
        if self.imageCounter!=0:
            return
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        # state : 0-red light; 1-yellow light; 2-green light
        
        #=== debug info====
        '''
        light_coord = self.waypoints_2d[light_wp]
        self_coord = [self.pose.pose.position.x, self.pose.pose.position.y]
        dist = np.sqrt((light_coord[0]-self_coord[0])**2 + (light_coord[1]-self_coord[1])**2)
        #rospy.logwarn('Next light is at waypoint %d, state=%d, distance to car= %.2f', light_wp, state, dist)
        strTruth = 'unknown'
        if self.closest_light:
            strTruth = '%d'%self.closest_light.state
        print('Next light is at waypoint %d, state=%d (actual state=%s), distance to car= %.2f'%(light_wp, state, strTruth, dist))
        '''
        #=== end debug info===

        ##==== save images, Comment this out when deploy!!!====
        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #ts = datetime.datetime.now()
        #filename = ts.strftime('%Y-%m-%d_%H_%M_%S')+'-state-%d-dist-%d'%(state, dist)+'.jpg'
        #cv2.imwrite('/home/student/myShare/'+filename, cv_image)
        ##==== end saving images

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement (Done)
        if self.waypoint_tree is None:
            return -1
        else:
            x = pose.position.x
            y = pose.position.y
            closest_idx = self.waypoint_tree.query([x, y], 1)[1]
            return closest_idx
            ''' the following seems unnecessary
            closest_coord = self.waypoints_2d[closest_idx]
            # tell if cloest_idx is in front or behind the vehicle
            prev_coord = self.waypoints_2d[(closest_idx-1+len(self.waypoints_2d)) % len(self.waypoints_2d)] # add len(self.waypoints_2d) in case the closest_idx happens to be 0
            val = np.dot(np.array(closest_coord)-np.array(prev_coord), np.array(closest_coord)-np.array([x, y])) 
            if val>=0:
                return closest_idx
            else:
                return (closest_idx+1) % len(self.waypoints_2d)
            '''
    
    

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #return 2
        #return light.state ## debug use only, this will return the ground truth light state
                           ## # state : 0-red light; 1-yellow light; 2-green light        

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image, self.config['is_site'])

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.closest_light = None  # to store the closest light position
        line_wp_idx = None  # to store the closest light stop line position (actually its closest waypoint)

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose)

            #TODO find the closest visible traffic light (if one exists) (done)
            diff = len(self.waypoints.waypoints) #diff to find min diff in index from car_position to closest_light
            for i, light in enumerate(self.lights):
                line = Pose()  #synthetic a Pose msg for calling get_closest_waypoint()
                line.position.x, line.position.y = stop_line_positions[i]  #assign values
                temp_wp_idx = self.get_closest_waypoint(line) #waypoint next to the traffic light stop line
                d = (temp_wp_idx-car_wp_idx + len(self.waypoints.waypoints)) % len(self.waypoints.waypoints)
                if d<diff:  #d is always >=0 from the above
                    diff = d
                    self.closest_light = light
                    line_wp_idx = temp_wp_idx                            

        if self.closest_light:
            state = self.get_light_state(self.closest_light)
            return line_wp_idx, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

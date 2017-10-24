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
import math

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 400
MAX_DISTANCE_STOP_TO_LIGHT = 50

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.current_waypoint_index = None
        self.lights = []
        # array of waypoint indexes indicating the nearest waypoint to the lights
        self.light_waypoints = None
        # array of indexes indicating the nearest stop line to the lights
        self.light_stops = None
        # array of waypoint indexes indicating the nearest waypoint to a stop line
        self.stop_waypoints = None
        # array of waypoint indexes indicating the nearest waypoint to a stop line
        self.init_done = False
        # get the maximum detection distance for the lights in m
        self.max_detect_distance = rospy.get_param('/tl_detector/max_detect_distance', 100)
        # simulation environment
        self.simulation = True

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_waypoint_index', Int32, self.current_waypoint_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        subImage = rospy.Subscriber('/image_color', Image, self.image_cb)

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

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def current_waypoint_cb(self, index):
        self.current_waypoint_index = index.data

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # only once search through the list of traffic lights and find their respective nearest waypoints
        if self.waypoints != None and self.init_done == False:
            self.init_traffic_light_waypoints()
            self.init_stop_line_waypoints()
            self.init_traffic_light_stops()
            self.init_done = True

    def init_traffic_light_waypoints(self):
        self.light_waypoints = []
        for light in self.lights:
            light_waypoint_index = self.get_closest_waypoint(light.pose.pose)
            rospy.loginfo("init_traffic_light_waypoints: traffic light (%s,%s,%s) nearest to waypoint %s",
                          light.pose.pose.position.x, light.pose.pose.position.y, light.pose.pose.position.z,
                          light_waypoint_index)
            self.light_waypoints.append(light_waypoint_index)

    def init_stop_line_waypoints(self):
        # only once search through the stop lines and find their respective nearest waypoints
        stop_line_positions = self.config['stop_line_positions']
        self.stop_waypoints = []
        for stop in stop_line_positions:
            stop_location = Pose()
            stop_location.position.x = stop[0]
            stop_location.position.y = stop[1]
            stop_waypoint_index = self.get_closest_waypoint(stop_location)
            rospy.loginfo("init_stop_line_waypoints: stop line (%s,%s,%s) nearest to waypoint %s",
                          stop_location.position.x, stop_location.position.y, stop_location.position.z,
                          stop_waypoint_index)
            self.stop_waypoints.append(stop_waypoint_index)

    def init_traffic_light_stops(self):
        # only once search through the light list and try to associate them with a stop line
        self.light_stops = []
        for light_index in range(len(self.light_waypoints)):
            self.light_stops.append(None)
            for stop_index in range(len(self.stop_waypoints)):
                distance = self.distanceBetweenWaypoints(self.light_waypoints[light_index],
                                                         self.stop_waypoints[stop_index])
                if distance < MAX_DISTANCE_STOP_TO_LIGHT:
                    rospy.loginfo("init_traffic_light_waypoints: stop %s nearest to traffic light %s",
                                  light_index, stop_index)
                    self.light_stops[light_index] = stop_index

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

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
        #TODO implement
        min_distance = 1e6
        wp_closest = None
        last_distance = None
        for wp_index in range(len(self.waypoints)):
            distance = self.distanceToWaypoint(pose, self.waypoints[wp_index].pose.pose)
            if distance < min_distance:
                min_distance = distance
                wp_closest = wp_index
            else:
                if last_distance != None and last_distance < distance and distance < self.max_detect_distance:
                    break
            last_distance = distance
        rospy.loginfo("get_closest_waypoint: wp_closet=%s distance=%s", wp_closest, min_distance)
        return wp_closest

    def distanceBetweenWaypoints(self, wp1_index, wp2_index):
        wp1 = self.waypoints[wp1_index]
        wp2 = self.waypoints[wp2_index]
        distance = self.distanceToWaypoint(wp1.pose.pose, wp2.pose.pose)
        return distance

    def distanceToWaypoint(self, pose, waypoint):
        distance = self.distanceBetweenPoints(pose.position, waypoint.position)
        return distance

    def distanceBetweenPoints(self, p1, p2):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        # calculate Pythagorean distance between waypoint and car's pose
        distance = dl(p1, p2)
        return distance

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        stop = None

        # List of positions that correspond to the line to stop in front of for a given intersection
#        stop_line_positions = self.config['stop_line_positions']
#        if(self.pose):
#            car_position = self.get_closest_waypoint(self.pose)

        #TODO find the closest visible traffic light (if one exists)
        if self.init_done and self.current_waypoint_index != None:
            for light_index in range(len(self.light_waypoints)):
                # get the waypoint index delta to our current waypoint for this light
                waypoint_index_delta = (self.light_waypoints[light_index] - self.current_waypoint_index +
                                        len(self.waypoints)) % len(self.waypoints)
                # calculate our distance to this light
                distanceToLight = self.distanceToWaypoint(self.waypoints[self.light_waypoints[light_index]].pose.pose,
                                                          self.waypoints[self.current_waypoint_index].pose.pose)
                # only proceed if the light is within the maximum detection range and the waypoint index is not too far away
                if distanceToLight < self.max_detect_distance and waypoint_index_delta < LOOKAHEAD_WPS:
                    rospy.loginfo("process_traffic_lights: traffic light %s %sm ahead of waypoint %s in state %s",
                                  light_index, distanceToLight, self.current_waypoint_index, self.lights[light_index].state)
                    light = light_index
                    stop_index = self.light_stops[light_index]
                    # check to see if this light has an associated stop line
                    if stop_index != None:
                        # get the waypoint index delta to our current waypoint for this stop line
                        waypoint_index_delta = (self.stop_waypoints[light_index] - self.current_waypoint_index +
                                                len(self.waypoints)) % len(self.waypoints)
                        # calculate our distance to this stop line
                        distanceToStop = self.distanceToWaypoint(
                            self.waypoints[self.stop_waypoints[light_index]].pose.pose,
                            self.waypoints[self.current_waypoint_index].pose.pose)
                        # only proceed if the stop line waypoint index is not too far away
                        if waypoint_index_delta < LOOKAHEAD_WPS:
                            rospy.loginfo("process_traffic_lights: stop %s %sm ahead of waypoint %s",
                                          stop_index, distanceToStop, self.current_waypoint_index)
                            stop = stop_index
                        else:
                            # we are passes the stop line already but before the light, just keep going
                            rospy.loginfo("process_traffic_lights: we are passed stop line %s keeping going", stop_index)
                            light = None
                    break;

        if light != None:
            # nearby light was detected, find out what state it is in
            state = self.get_light_state(light)
            # for the simulation case use the supplied light state
            if state == TrafficLight.UNKNOWN and self.simulation:
                state = self.lights[light].state
            # pass back the stop line waypoint index if there is one for this light
            if stop != None:
                light_wp = self.stop_waypoints[stop]
            else:
                light_wp = self.light_waypoints[light]
            rospy.loginfo("process_traffic_lights: waypoint=%s state=%s", light_wp, state)
            return light_wp, state
#        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

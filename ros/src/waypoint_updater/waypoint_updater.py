#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf
import numpy as np

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoints', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoints', Lane, self.obstacle_cb, queue_size=1)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # get the speed limit in kph from the waypoint_loader node params store in ms-1
        self.speed_limit = rospy.get_param('/waypoint_loader/velocity', 10) * (1000 / 3600)
        # get the main message loop update rate in Hz from the waypoint_updater node params
        self.update_rate = rospy.get_param('/waypoint_updater/update_rate', 10)
        # empty placeholder for base waypoints
        self.base_waypoints = None
        # empty placeholder for the current pose of the vehicle
        self.current_pose = None
        # empty placeholder for the current nearest waypoint
        self.current_wp = None
        # extra debug option
        self.extraDebug = False

        self.loop()

    def loop(self):
        # main message loop for waypoint updater
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            # do we have the base waypoints yet?
            if self.base_waypoints and self.current_pose != None:
                # find out which waypoint we are next nearest to
                wp_closest = self.locateNextWaypoint()

                # build the list of next waypoints
                waypoints = []
                if wp_closest != None:
                    for wp in range(LOOKAHEAD_WPS):
                        wp_index = (wp + wp_closest) % len(self.base_waypoints)
                        waypoints.append(self.base_waypoints[wp_index])
#                rospy.loginfo("WaypointUpdater: waypoints=%s", waypoints)
                self.publish(waypoints)
            rate.sleep()

    def publish(self, waypoints):
        # populate the next_waypoint message
        next_waypoints = Lane()
        next_waypoints.header.frame_id = '/word'
        next_waypoints.header.stamp = rospy.Time.now()
        next_waypoints.waypoints = waypoints
        # publish the waypoints
        self.final_waypoints_pub.publish(next_waypoints)

    def locateNextWaypoint(self):
        min_distance = 1e6
        wp_closest = None
        wp_offset = 0
        last_distance = None
        if self.current_wp != None:
            wp_offset = self.current_wp
        for wp in range(len(self.base_waypoints)):
            wp_index = (wp + wp_offset - 2) % len(self.base_waypoints)
            distance = self.distanceCarToWaypoint(self.current_pose, self.base_waypoints[wp_index].pose.pose)
            if self.extraDebug:
                rospy.loginfo("locateNextWaypoint: index=%s wp(%s,%s,%s) car(%s,%s,%s) distance=%s", wp_index,
                              self.base_waypoints[wp_index].pose.pose.position.x,
                              self.base_waypoints[wp_index].pose.pose.position.y,
                              self.base_waypoints[wp_index].pose.pose.position.z,
                              self.current_pose.position.x,
                              self.current_pose.position.y,
                              self.current_pose.position.z,
                              distance)
            if distance < min_distance:
                min_distance = distance
                wp_closest = wp_index
                if self.extraDebug:
                    rospy.loginfo("locateNextWaypoint: wp_index=%s last_distance=%s distance=%s", wp_index, last_distance, distance)
            else:
                if last_distance != None and last_distance < distance:
                    break
                last_distance = distance

        rospy.loginfo("locateNextWaypoint: wp_closet=%s distance=%s", wp_closest, min_distance)

        # store this nearest waypoint for next time - probably no good if the car goes backwards!
        self.current_wp = wp_closest
        return wp_closest

    def radToDeg(self, rad):
        return 180 * rad / np.pi

    def distanceCarToWaypoint(self, pose, waypoint):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        # calculate Pythagorean distance between waypoint and car's pose
        distance = dl(pose.position, waypoint.position)
        return distance

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose
#        rospy.loginfo("pose_cb: %s", msg.pose)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo("waypoints_cb: received %s base waypoints", len(waypoints.waypoints))
        self.base_waypoints = waypoints.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        self.base_wp = None
        self.base_wp_cnt = 0
        self.final_wp = None
        self.pos = None
        self.closest_wp_idx = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.update()
        rospy.spin()
    
    def update(self):
        if self.base_wp == None or self.pos == None:
            return
        closest_wp_idx = self.get_closest_waypoint(self.pos, self.base_wp)
        self.publish_final_waypoints(closest_wp_idx)

    def pose_cb(self, msg):
        '''
        Callback for /traffic_waypoint message. Implement
        :param msg: geometry_msgs/PoseStamped with current position of vehicle
        '''
        # get nearest waypoints and publish them 
        rospy.loginfo('New position %f', msg.pose.position.x)
        closest_wp_idx = self.get_closest_waypoint(msg.pose.position, self.base_wp)
        rospy.loginfo('Closest waypoint idx %i', closest_wp_idx)
        # self.final_waypoints_pub.publish(final_wp)
        self.pos = msg

    def waypoints_cb(self, waypoints):
        '''
        Callback to get all waypoints, this is published only once 
        :param waypoints: styx_msgs/Lane static waypoints provided by csv
        '''
        rospy.loginfo('Got %i base waypoints', len(waypoints.waypoints))
        self.base_wp = waypoints.waypoints
        self.base_wp_cnt = len(self.base_wp)

    def traffic_cb(self, msg):
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

    def distance(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def get_closest_waypoint(self, pos, waypoints):
        closest_len = 1000000 # large umber
        closest_idx = 0
        for i,w in enumerate(waypoints):
            dist = self.distance(pos, w.pose.pose.position)
            if dist < closest_len:
                closest_len = dist
                closest_idx = i
        return closest_idx

    def publish_final_waypoints(self, wp_idx):
        final_waypoints = Lane()
        final_waypoints.header.stamp = self.pos.header.stamp
        final_waypoints.header.frame_id = self.pos.header.frame_id
        # if self.closest_wp_idx + LOOKAHEAD_WPS >= self.base_wp_cnt:
        #     final_waypoints.waypoints

        last_idx = self.closest_wp_idx + LOOKAHEAD_WPS
        final_waypoints.waypoints = self.base_wp[self.closest_wp_idx:last_idx+1]
        self.final_waypoints_pub(final_waypoints)
        


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

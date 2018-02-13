#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DISTANCE_AHEAD = 700 # Maximum distance ahead needed to look


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # Using the data from the current position, provide waypoints ahead
        # create variables obtaining the placement of the vehicle
        cx_position = msg.pose.orientation.x
        cy_position = msg.pose.orientation.y
        cz_position = msg.pose.orientation.z
        cw_position = msg.pose.orientation.w
        # Find the waypoints in the base waypoints that are after the current position and less than 70 m away
        # the points will need to be transformed into the vehicle's coordinate space
        self.oncoming_waypoints = Lane()
        self.oncoming_waypoints_distance = []
        for each_waypoint in self.base_waypoints.waypoints:
            #create variables for the placement of the waypoint
            each_waypointx = each_waypoint.pose.pose.orientation.x
            each_waypointy = each_waypoint.pose.pose.orientation.y
            each_waypointz = each_waypoint.pose.pose.orientation.z
            # transform the waypoint
            shift_x = each_waypointx - cx_position
            shift_y = each_waypointy - cy_position
            each_waypointx = shift_x * math.cos(0-cw_position) - shift_y * math.sin(0-cw_position)
            each_waypointy = shift_x * math.sin(0-cw_position) + shift_y * math.cos(0-cw_position)
            # obtain the distance
            waypoint_distance = ((cx_position-each_waypointx)**2 + (cx_position-each_waypointx)**2 + (cx_position-each_waypointx)**2 * 1.0)**(0.5)
            #if the waypoint is in proximity of the vehicle and in front of the vehicle
            if (waypoint_distance<DISTANCE_AHEAD and each_waypointx>0):
                # add to the oncoming waypoints
                self.oncoming_waypoints.waypoints.append(each_waypoint)
                # add to the distance list holder
                self.oncoming_waypoints_distance.append(waypoint_distance)
        # The callback_function for the '/current_pose' provides oncoming_waypoints
        # and their distances from the '/current_pose'. Put them into self.final_waypoints
        # by smallest distance to largest distance.
        # obtain a sorted list of indices from the distances
        self.oncoming_waypoints_distance_sorted = np.array(self.oncoming_waypoints_distance).argsort()[:LOOKAHEAD_WPS].astype(int).tolist()
        # create a final_waypoints
        self.final_waypoints = Lane()
        # add the waypoints to the final_waypoints with respect to the sorted distance
        for each_index in self.oncoming_waypoints_distance_sorted:
            self.final_waypoints.waypoints.append(self.oncoming_waypoints.waypoints[each_index])
        self.final_waypoints_pub.publish(self.final_waypoints)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
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

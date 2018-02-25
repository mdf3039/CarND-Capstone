#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float64

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
MAX_ACCELERATION = 9.0 
MAX_JERK = 9.0
MAX_DECELERATION = 5.0


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        #Get the maximum velocity parameter
        self.maximum_velocity = self.kmph2mps(rospy.get_param('~velocity')) # change km/h to m/s and subtract 1 to make sure it is always lower

        #Set an intial for a previous waypoint index
        self.stopping_waypoint_index = -1
        #Set the previous velocities
        self.previous_previous_velocity = 0
        self.previous_velocity = 0
        self.base_waypoints = None
        self.oncoming_waypoints_distance = []
        self.transformed_xy = []
        self.oncoming_waypoints = None

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        self.cte_pub = rospy.Publisher('/cross_track_error',Float64, queue_size=1)

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        rospy.spin()
        # TODO: Add other member variables you need below

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def current_velocity_function(self,msg):
        # obtain current_velocity for yaw controller
        self.current_velocity = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2 * 1.0)**(1.0/2)
        #obtain current_angular_velocity for controller
        self.current_angular_velocity = (msg.twist.angular.x**2 + msg.twist.angular.y**2 + msg.twist.angular.z**2 * 1.0)**(1.0/2)
        # pass

    def pose_cb(self, msg):
        # TODO: Implement
        # Using the data from the current position, provide waypoints ahead
        # create variables obtaining the placement of the vehicle
        cx_position = msg.pose.position.x
        cy_position = msg.pose.position.y
        cz_position = msg.pose.position.z
        cw_position = msg.pose.orientation.w
        # Find the waypoints in the base waypoints that are after the current position and less than 70 m away
        # the points will need to be transformed into the vehicle's coordinate space
        self.oncoming_waypoints = Lane()
        self.oncoming_waypoints_distance = []
        self.transformed_xy = []
        self.two_closest_waypoints = np.empty((0,3), float)
        # If the Base Waypoints have not been uploaded yet, do not start
        if self.base_waypoints is None:
            rospy.loginfo("THE BASE WAYPOINTS ARE NOT THERE")
            return
        rospy.loginfo("THE BASE WAYPOINTS ARE FOUND")
        for each_waypoint in self.base_waypoints:
            #create variables for the placement of the waypoint
            each_waypointx = each_waypoint.pose.pose.position.x
            each_waypointy = each_waypoint.pose.pose.position.y
            each_waypointz = each_waypoint.pose.pose.position.z
            # transform the waypoint
            shift_x = each_waypointx - cx_position
            shift_y = each_waypointy - cy_position
            each_waypointx = shift_x * math.cos(0-cw_position) - shift_y * math.sin(0-cw_position)
            each_waypointy = shift_x * math.sin(0-cw_position) + shift_y * math.cos(0-cw_position)
            # obtain the distance
            waypoint_distance = (each_waypointx**2 + each_waypointy**2 * 1.0)**(0.5)
            #if the waypoint is in proximity of the vehicle and in front of the vehicle
            if (waypoint_distance<DISTANCE_AHEAD and each_waypointx>0):
                # add to the oncoming waypoints
                self.oncoming_waypoints.waypoints.append(each_waypoint)
                # add to the distance list holder
                self.oncoming_waypoints_distance.append(waypoint_distance)
                #add the transformed x and y to a list to store the transformed x and y. Use to make polynomial fitting later 
                #self.transformed_xy.append([each_waypointx,each_waypointy])
            #for the cross track error, keep the two waypoints that are closest to the current position
            #record the distance, x, and y for the waypoints
            self.two_closest_waypoints = np.append(self.two_closest_waypoints, np.array([[waypoint_distance,each_waypointx,each_waypointy]]), axis=0)
            self.two_closest_waypoints = self.two_closest_waypoints[self.two_closest_waypoints[:,0].argsort()[:2]]
        #Find the distance from the line segment of the two closest points and the current position(0,0)
        self.cross_track_error = self.two_closest_waypoints[0,2] - self.two_closest_waypoints[0,1]*(self.two_closest_waypoints[0,2]-self.two_closest_waypoints[1,2])/(self.two_closest_waypoints[0,1]-self.two_closest_waypoints[1,1])
        #fit the polynomial
        #self.transformed_xy = np.array(self.transformed_xy)
        #poly_output = np.poly1d(np.polyfit(self.transformed_xy[:,0].tolist(), self.transformed_xy[:,1].tolist(), 3))
        #untransform the points
        #for 
        # sort oncoming waypoints with respect to the distance from the current position
        self.oncoming_waypoints_distance_sorted = np.array(self.oncoming_waypoints_distance).argsort()[:LOOKAHEAD_WPS].astype(int).tolist()
        # create a final_waypoints
        self.final_waypoints = Lane()
        # add the waypoints to the final_waypoints with respect to the sorted distance. Also change the speed to the max_velocity
        for each_index in self.oncoming_waypoints_distance_sorted:
            self.final_waypoints.waypoints.append(self.oncoming_waypoints.waypoints[each_index])
            #Also change the speed to the max_velocity
            self.final_waypoints.waypoints[-1].twist.twist.linear.x = 8#self.maximum_velocity
        self.final_waypoints_pub.publish(self.final_waypoints)
        rospy.loginfo("The CTE in wpt_updtr: " + str(self.cross_track_error))
        self.cte_pub.publish(self.cross_track_error)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # rospy.loginfo("Oncoming Waypoints are loading")
        self.base_waypoints = waypoints.waypoints
        # rospy.loginfo("The number of oncoming waypoints are: " + str(len(waypoints.waypoints)))

    def traffic_cb(self, msg):
        #choose the model, depending upon the msg
        msg = int(str(msg))
        if msg==-2:
            # Unknown traffic light. Use previous model
            None
        if msg==-1:
            # Green light or far distance from red model
            self.mpc_model = self.ModelGo
            self.stopping_waypoint_index = msg
        if msg>=0:
            # Red or Yellow light model
            self.mpc_model = self.ModelStop
            self.stopping_waypoint_index = msg

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

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
    
    ModelStop = 1
    ModelGo = 2
    
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
        self.c_position = None
        self.prev_pose = None

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        # self.cte_pub = rospy.Publisher('/cross_track_error',Float64, queue_size=1)

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb_function)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # self.traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # self.loop()
        # TODO: Add other member variables you need below


    def loop(self):
        rate = rospy.Rate(10) # 1Hz
        while not rospy.is_shutdown():
            rospy.loginfo("THE current position: " + str(self.c_position))
            self.pose_cb(self.c_position)
            rospy.loginfo("Finished")
            rate.sleep()

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def current_velocity_function(self,msg):
        # obtain current_velocity for yaw controller
        self.current_velocity = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2 * 1.0)**(1.0/2)
        #obtain current_angular_velocity for controller
        self.current_angular_velocity = (msg.twist.angular.x**2 + msg.twist.angular.y**2 + msg.twist.angular.z**2 * 1.0)**(1.0/2)
        # pass
    
    def pose_cb_function(self, msg):
        self.c_position = np.array([msg.pose.position.x, msg.pose.position.y])

    def pose_cb(self, msg):
        if msg is None:
            return
        if self.base_waypoints is None:
            # rospy.loginfo("THE BASE WAYPOINTS ARE NOT THERE")
            return
        # TODO: Implement
        if self.prev_pose is None:
            self.prev_pose = msg - 0.1
        if np.all(self.prev_pose == msg):
            return
        # Find the waypoints in the base waypoints that are after the current position and less than 70 m away
        # obtain the distance then use the sign of the dot product
        waypoint_distances = np.sqrt(((self.base_waypoints - msg)**2).sum(axis=1))
        # Find the sign of the dot product of the position vector and the base waypoints wrt the previous position
        dot_product_sign = np.sign(np.dot(self.base_waypoints-self.prev_pose, msg-self.prev_pose))
        # Multiply the waypoint_distances by their respective sign. Positive distances mean in front of car
        waypoint_distances = np.multiply(waypoint_distances,dot_product_sign)
        # obtain the indices of the smallest positive LOOKAHEAD_WPS. Set all negative distance values to 1,000,000 to make easier.
        waypoint_distances[np.where(waypoint_distances<0)[0]] = 1000000
        indices = waypoint_distances.argsort()[:LOOKAHEAD_WPS].astype(int).tolist()
        # create a final_waypoints
        self.final_waypoints = Lane()
        # add the waypoints to the final_waypoints with respect to the sorted distance.
        for each_index in indices:
            self.final_waypoints.waypoints.append(self.wpts[each_index])
        self.final_waypoints_pub.publish(self.final_waypoints)
        # make the msg the prev_pose
        self.prev_pose = msg.copy()

    def waypoints_cb(self, msg):
        self.wpts = msg.waypoints
        # TODO: Implement
        base_waypoints = []
        for each_waypoint in msg.waypoints:
            base_waypoints.append([each_waypoint.pose.pose.position.x, each_waypoint.pose.pose.position.y])
        self.base_waypoints = np.array(base_waypoints)

    def traffic_cb(self, msg):
        #choose the model, depending upon the msg
        msg = msg.data
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

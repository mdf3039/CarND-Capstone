#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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
        self.previous_waypoint_index = -1
        #Set the previous velocities
        self.previous_previous_velocity = 0
        self.previous_velocity = 0
        self.base_waypoints = None
        self.oncoming_waypoints_distance = []
        self.transformed_xy = []
        self.oncoming_waypoints = None

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        # TODO: Add other member variables you need below
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # The callback_function for the '/current_pose' provides oncoming_waypoints
            # and their distances from the '/current_pose'. Put them into self.final_waypoints
            # by smallest distance to largest distance.
            # obtain a sorted list of indices from the distances
            if self.oncoming_waypoints is None:
                rospy.loginfo("Oncoming Waypoints are not there")
                continue
            rospy.loginfo("Oncoming Waypoints arrived")
            self.oncoming_waypoints_distance_sorted = np.array(self.oncoming_waypoints_distance).argsort()[:LOOKAHEAD_WPS].astype(int).tolist()
            # create a final_waypoints
            self.final_waypoints = Lane()
            # add the waypoints to the final_waypoints with respect to the sorted distance
            for each_index in self.oncoming_waypoints_distance_sorted:
                self.final_waypoints.waypoints.append(self.oncoming_waypoints.waypoints[each_index])
            rospy.loginfo(self.final_waypoints)
            # using the final waypoints, separate them out at a speed of maximum_velocity. 
            # fit a polynomial with transformed points

            self.final_waypoints_pub.publish(self.final_waypoints)
            rate.sleep()

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
        cx_position = msg.pose.orientation.x
        cy_position = msg.pose.orientation.y
        cz_position = msg.pose.orientation.z
        cw_position = msg.pose.orientation.w
        # Find the waypoints in the base waypoints that are after the current position and less than 70 m away
        # the points will need to be transformed into the vehicle's coordinate space
        self.oncoming_waypoints = Lane()
        self.oncoming_waypoints_distance = []
        self.transformed_xy = []
        #print ("The BASE WAYPOINTS ARE OF TYPE: ", type(self.base_waypoints))
        if self.base_waypoints is None:
            print "THE BASE WAYPOINTS ARE NOT THERE"
            self.current_pose = msg
            return
        for each_waypoint in self.base_waypoints:
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
                #add the transformed x and y to a list to store the transformed x and y. Use to make polynomial fitting later 
                self.transformed_xy.append([each_waypointx,each_waypointy])
        #fit the polynomial
        self.transformed_xy = np.array(self.transformed_xy)
        #poly_output = np.poly1d(np.polyfit(self.transformed_xy[:,0].tolist(), self.transformed_xy[:,1].tolist(), 3))
        #untransform the points
        #for 

        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        dist_from_pos = []
        # Set the current velocity to the previous, previous to the previous_previous
        self.previous_previous_velocity = self.previous_velocity
        self.previous_velocity = self.current_velocity
        #traffic light msg is either -2, -1, or the stopping index for the stopline to stop at
        #if msg is -2, this means the traffic light is unknown. Use the previous traffic_light_message
        if msg==-2:
            msg = self.previous_waypoint_index
        #If the msg is -1, this means pursue at full throttle (green light)
        if msg==-1:
            #set waypoints with respect to the current velocity and accelerate if needed
            #the next velocity cannot exceed a certain acceleration or jerk threshold. See which 
            #threshold is the smaller. The velocity cannot also be larger than the maximum velocity
            max_v_acceleration = MAX_ACCELERATION*.2 + self.previous_velocity
            max_v_jerk = .04*MAX_JERK + 2*self.previous_velocity - self.previous_previous_velocity
            max_v = min(max_v_jerk, max_v_acceleration, self.maximum_velocity)
            #obtain the distance from the current position. 
            dist_from_pos.append(.1*(max_v+self.previous_velocity))
            #add on the other distances for the list
            for i in range(LOOKAHEAD_WPS-1):
                #if the maximum velocity will be reached in the next acceleration burst, or is the current velocity
                if (self.maximum_velocity-max_v)/.2 <= MAX_ACCELERATION:
                    dist_from_pos.append(dist_from_pos[-1] + .1*(max_v+self.maximum_velocity))
                    max_v = self.maximum_velocity
                else:
                    dist_from_pos.append(dist_from_pos[-1] + .2*max_v + .5*MAX_ACCELERATION*(.2**2))
                    max_v += MAX_ACCELERATION*.2
        #The message is a waypoint to stop at. Set a course to stop
        else:
            #Obtain waypoint
            stop_line = self.base_waypoints[msg]
            #Find the distance to the waypoint
            distance_to_stop_line = ((stop_line.pose.pose.orientation.x-self.current_pose.pose.orientation.x)**2 + (stop_line.pose.pose.orientation.y-self.current_pose.pose.orientation.y)**2 * 1.0)**(.5)
            if (self.current_velocity==0 or distance_to_stop_line<=9):
                #If the velocity is zero, then the line is close. Move at a speed of 1m/s towards the line
                for i in range(min(LOOKAHEAD_WPS,int(math.floor(distance_to_stop_line/0.2)))):
                    dist_from_pos.append((i+1)*.2)
                #complete whatever is left with the last entry (thereby not moving)
                for i in range(LOOKAHEAD_WPS-len(dist_from_pos)):
                    dist_from_pos.append(dist_from_pos[-1])
            else:
                #decrease speed. The next velocity cannot exceed a certain acceleration or jerk
                #threshold. See which threshold is smaller. The velocity cannot also be larger than the maximum velocity
                min_v_acceleration = -1*MAX_DECELERATION*.2 + self.previous_velocity
                min_v_jerk = .04*-1*MAX_JERK + 2*self.previous_velocity - self.previous_previous_velocity
                min_v = max(min_v_jerk, min_v_acceleration, 0)
                #obtain the distance from the current position. 
                dist_from_pos.append(.1*(min_v+self.previous_velocity))
                #add on the other distances for the list
                for i in range(LOOKAHEAD_WPS-1):
                    #if the velocity is zero, append the previous entry
                    if min_v==0:
                        dist_from_pos.append(dist_from_pos[-1])
                    #if the target velocity will be reached in the next deceleration burst, decelerate at a rate of 1m/s
                    if (3-min_v)/.2 >= -1*MAX_DECELERATION:
                        dist_from_pos.append(dist_from_pos[-1] + .1*(min_v+self.maximum_velocity))
                        min_v = self.maximum_velocity
                    else:
                        dist_from_pos.append(dist_from_pos[-1] + .2*min_v + .5*MAX_ACCELERATION*(.2**2))
                        min_v += MAX_ACCELERATION*.2
        # pass

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

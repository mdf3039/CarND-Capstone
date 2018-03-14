#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64, Int32
from styx_msgs.msg import Lane, TrafficLightArray
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from tl_detector import TLDetector
import math
import numpy as np

from twist_controller import Controller
from pid import PID

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        
        # TODO: Subscribe to all the topics you need to
        self.maximum_velocity = self.kmph2mps(rospy.get_param('~velocity')) # change km/h to m/s and subtract 1 to make sure it is always lower
        # self.maximum_velocity = 20
        self.cte = 0
        self.cte_bool = False
        self.prev_sample_time = None
        self.current_velocity = 0
        self.current_angular_velocity = 0
        #self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.steer_direction = 0
        self.base_waypoints = None
        self.prev_position = None
        self.prev_msg = np.array([-1 , -1])
        self.prev_midpoint = None
        self.prev_prev_midpoint = None
        self.two_closest_points = None
        self.prev_light_msg = -1
        self.light_msg = -1
        self.drive_model = -1
        self.prev_cte = 0
        self.cte_sign = 1
        kp = 0.0 # or try these values:
        ki = 0.0 # kp=0.3, ki=0.0, kd=0.57
        kd = 0.0
        self.c_position = None
        self.pid_controller_cte = PID(kp, ki, kd)
        self.pid_controller_angle = PID(kp, ki, kd)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)
        # self.cte_sub = rospy.Subscriber('/cross_track_error',Float64, self.cte_function)
        #self.twist_cmd_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_function)
        self.dbw_enabled_bool = False
        self.dbw_enabled_sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_function)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb_function)
        self.traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # obtain min_speed for the yaw controller by adding the deceleration times time to the current velocity
        self.min_speed = 0 #max(0, decel_limit*time + self.current_velocity(needs to be finished))

        # TODO: Create `Controller` object
        # The Controller object returns the throttle and brake.
        self.controller = Controller(self.wheel_base, self.steer_ratio, self.min_speed, max_lat_accel, max_steer_angle, self.vehicle_mass, self.wheel_radius)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        #place the TLDETECTOR in the dbw for analysis
        self.tl_detector = TLDetector()
        self.loop_rate = 5
        self.loop() 
        # rospy.spin()

    def loop(self):
        rate = rospy.Rate(self.loop_rate) # 1Hz
        while not rospy.is_shutdown():
            self.traffic_cb(self.tl_detector.actual_image_test(rospy.wait_for_message('/vehicle/traffic_lights', TrafficLightArray)))
            self.pose_cb(self.c_position)
            rate.sleep()
    
    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # rospy.loginfo("Oncoming Waypoints are loading")
        self.base_waypoints = []
        for waypoint in waypoints.waypoints:
            # add to the waypoints list
            self.base_waypoints.append([waypoint.pose.pose.position.x, waypoint.pose.pose.position.y])
        self.base_waypoints = np.array(self.base_waypoints)
        # rospy.loginfo("The number of oncoming waypoints are: " + str(self.base_waypoints.shape))

    def pose_cb_function(self, msg):
        self.c_position = msg

    def pose_cb(self, msg):
        if msg is None:
            return
        if self.base_waypoints is None:
            return
        rospy.loginfo("Position is updated: " + str(msg.pose.position.x) + "," + str(msg.pose.position.y))
        msg = np.array([msg.pose.position.x, msg.pose.position.y])
        if msg[0]==self.prev_msg[0] and msg[1]==self.prev_msg[1]:
            return
        #Find the closest two waypoints given the position.
        self.steer = 0
        if self.prev_sample_time is None:
            self.sample_time = 0.2
            self.prev_sample_time = rospy.get_time()
        else:
            time = rospy.get_time()
            self.sample_time = time - self.prev_sample_time
            # rospy.loginfo("Delta Time: " + str(self.sample_time))
            self.prev_sample_time = time
        if self.base_waypoints is not None:
            if msg[0]==self.prev_msg[0] and msg[1]==self.prev_msg[1]:
                return
            #the distances from the current position for all waypoints
            wp_distances = ((self.base_waypoints-msg)**2).sum(axis=1)
            #find and append the closest, fourth, and eighth point
            circle_points = self.base_waypoints[np.argmin(wp_distances)]
            # rospy.loginfo("circle_points: " + str(circle_points))
            circle_points = np.vstack((circle_points, self.base_waypoints[(np.argmin(wp_distances)+5)%len(wp_distances)]))
            # rospy.loginfo("circle_points: " + str(circle_points.shape))
            circle_points = np.vstack((circle_points, self.base_waypoints[(np.argmin(wp_distances)+10)%len(wp_distances)]))
            # rospy.loginfo("circle_points: " + str(circle_points.shape))
            #use the three points to find the radius of the circle
            eval_matrix = np.vstack((-2*circle_points[:,0],-2*circle_points[:,1],(circle_points**2).sum(axis=1))).T
            # rospy.loginfo("eval_matrix: " + str(eval_matrix.shape))
            #subtract the last entry of the eval matrix from the others and keep the first two rows
            eval_matrix = np.subtract(eval_matrix,eval_matrix[2])[0:2]
            try:
                x = np.linalg.solve(eval_matrix[:,0:2],eval_matrix[:,2])
                # rospy.loginfo("X obtained: " + str(x))
                radius = (((msg-x)**2).sum()*1.0)**(1.0/2)
                # rospy.loginfo("Radius: " + str(radius))
                #convert the angle into degrees then divide by the steering ratio to get the steer value
                angle = np.arcsin(self.wheel_base/radius) #* (180.0/np.pi)
                steer_value = angle * self.steer_ratio
            except:
                steer_value = 0
            #the sign of the steer value depends on the slope from the first to the last point
            two_closest_points = self.base_waypoints[np.sort(wp_distances.argsort()[:2])].copy()
            # make sure the waypoints are in the correct order by comparing their distance from the previous midpoint
            # keep the last two midpoints
            if self.prev_midpoint is None:
                self.two_closest_points = two_closest_points.copy()
                self.prev_midpoint = np.divide(np.add(two_closest_points[0],two_closest_points[1]),2.0).copy()
                self.prev_prev_midpoint = self.prev_midpoint.copy()
            elif np.all(self.prev_midpoint==np.divide(np.add(two_closest_points[0],two_closest_points[1]),2.0)):
                two_closest_points = self.two_closest_points.copy()
            else:
                #if the midpoints are not equal, sort by proximity to the previous midpoint
                # rospy.loginfo("Closest points may change: " + str(two_closest_points[0][0]) + "," + str(two_closest_points[0][1]))
                # rospy.loginfo("Closest points may change: " + str(two_closest_points[1][0]) + "," + str(two_closest_points[1][1]))
                self.two_closest_points = two_closest_points[((two_closest_points-self.prev_midpoint)**2).sum(axis=1).argsort()].copy()
                two_closest_points = self.two_closest_points.copy()
                self.prev_prev_midpoint = self.prev_midpoint.copy()
                self.prev_midpoint = np.divide(np.add(two_closest_points[0],two_closest_points[1]),2.0).copy()
            # rospy.loginfo("Closest points: " + str(two_closest_points[0][0]) + "," + str(two_closest_points[0][1]))
            # rospy.loginfo("Closest points: " + str(two_closest_points[1][0]) + "," + str(two_closest_points[1][1]))
            # get the direction for the steer value
            if (np.cross(two_closest_points[0]-self.prev_prev_midpoint,circle_points[2]-self.prev_prev_midpoint)<0):
                steer_value *= -1
            if np.all(self.prev_prev_midpoint == self.prev_midpoint):
                steer_value *= -1
            self.cte = abs(np.linalg.norm(np.cross(two_closest_points[0]-two_closest_points[1], two_closest_points[1]-msg))/np.linalg.norm(two_closest_points[0]-two_closest_points[1]))
            # rospy.loginfo("The CTE: " + str(self.cte))
            #the cross product will determine the direction. if the cross product is positive, the the car is to the left, cte is negative
            # rospy.loginfo("two_closest_points[0]-self.prev_prev_midpoint: " + str(two_closest_points[0]-self.prev_prev_midpoint))
            # rospy.loginfo("msg-self.prev_prev_midpoint: " + str(msg-self.prev_prev_midpoint))
            # rospy.loginfo("self.prev_prev_midpoint: " + str(self.prev_prev_midpoint))
            # rospy.loginfo("np.cross: " + str(np.cross(two_closest_points[0]-self.prev_prev_midpoint,msg-self.prev_prev_midpoint)))
            if (np.cross(two_closest_points[0]-self.prev_prev_midpoint,msg-self.prev_prev_midpoint)>0):
                self.cte *= -1
            if np.all(self.prev_prev_midpoint == self.prev_midpoint):
                self.cte *= -1
            # if ((course_midpoint-msg)**2).sum() < ((course_midpoint-self.prev_midpoint)**2).sum():
            # rospy.loginfo("The CTE: " + str(self.cte))
            kp_cte = 0.25#0.1 - .05*self.current_velocity/self.maximum_velocity###07 best is 0.31, .41
            ki_cte = 0.0#16#.08 # 1.015
            kd_cte = 0.5#0.25 + .20*self.current_velocity/self.maximum_velocity#5#.35 # 0.5
            pid_step_cte = max(min(self.pid_controller_cte.step(self.cte, self.sample_time, kp_cte, ki_cte, kd_cte), 8), -8)
            self.prev_msg = msg
            # rospy.loginfo("The steer value: " + str(steer_value))
            # rospy.loginfo("The PID CTE: " + str(pid_step_cte))
            # rospy.loginfo("The STR: " + str(steer_value+pid_step_angle+pid_step_cte))
            # the drive model will determine the throttle and brake
            if self.drive_model==-2:
                throttle, brake = 0, 0
                self.drive_model = self.prev_light_msg
            elif self.drive_model == -1:
                if self.current_velocity >= self.maximum_velocity:
                    throttle, brake = 0, 0
                else:
                    #accelerate at 8m/s**2. I noticed that at a constant throttle of 0.1, a velocity close to 12mph (5.36m/s) was reached.
                    #Using this as a constant proportion, accelerating 8m/s would require and extra .15 added to the throttle.
                    #its current throttle can be estimated by proportioning (.1/5.36) it to the current velocity.
                    throttle, brake = min(.42,self.current_velocity*.1/5.36 + .15), 0
            elif self.drive_model >= 0:
                rospy.loginfo("The self.drive_model: " + str(self.drive_model))
                rospy.loginfo("The self.drive_model type : " + str(type(self.drive_model)))
                #brake at a deceleration rate of current_velocity**2/(2*distance)
                wp_2_pos = ((msg-self.base_waypoints[int(self.drive_model)])**2).sum() - self.current_velocity*1.0/self.loop_rate
                brake_rate = self.current_velocity**2/(2*wp_2_pos)
                throttle, brake = 0, self.vehicle_mass*brake_rate*self.wheel_radius
                if self.current_velocity<5:
                    throttle, brake = 0, self.vehicle_mass*5
            # throttle, brake = self.controller.control(self.min_speed, self.linear_velocity, self.angular_velocity, 
            #                                                                     self.current_velocity, self.current_angular_velocity)
            if self.dbw_enabled_bool:
                # rospy.loginfo("The steer: " + str(steer_value+pid_step_cte))
                self.publish(throttle=throttle, brake=brake, steer=steer_value+pid_step_cte)
                rospy.loginfo("The controls published: " + str(brake))
    
    def traffic_cb(self, msg):
        #choose the model, depending upon the msg
        try:
            self.light_msg = msg.data
        except:
            self.light_msg = msg
        if self.light_msg==-2:
            # Unknown traffic light. Use previous model
            self.light_msg = self.prev_light_msg
        if self.light_msg==-1:
            # Green light or far distance from red model
            #If the previous message was red, do not do anything
            if self.prev_light_msg>=0:
                self.drive_model = -2
            else:
                #use the green drive model
                self.drive_model = self.light_msg
        elif self.light_msg>=0:
            # Red or Yellow light model
            #If the previous message was green, do not do anything
            if self.prev_light_msg==-1:
                self.drive_model = -2
            else:
                #use the red drive model
                self.drive_model = self.light_msg
        self.prev_light_msg = self.light_msg
        rospy.loginfo("The self.drive_model: " + str(self.drive_model))

    def dbw_enabled_function(self,msg):
        self.dbw_enabled_bool =  msg.data
        self.dbw_enabled = msg

    def current_velocity_function(self,msg):
        # rospy.loginfo("Current velocity is loading")
        # obtain current_velocity for yaw controller
        self.current_velocity = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2 * 1.0)**(1.0/2)
        # rospy.loginfo("The current velocity is: " + str(self.current_velocity))
        #obtain current_angular_velocity for controller
        self.current_angular_velocity = (msg.twist.angular.x**2 + msg.twist.angular.y**2 + msg.twist.angular.z**2 * 1.0)**(1.0/2)
        # rospy.loginfo("The current angular velocity is: " + str(self.current_angular_velocity))
        # pass

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()

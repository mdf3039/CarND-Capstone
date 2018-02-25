#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64
from styx_msgs.msg import Lane
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import math

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

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        
        # TODO: Subscribe to all the topics you need to
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
        kp = 7.85
        ki = 0 # 1.015
        kd = 0 # 0.5
        self.pid_controller = PID(kp, ki, kd)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)
        self.cte_sub = rospy.Subscriber('/cross_track_error',Float64, self.cte_function)
        #self.twist_cmd_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_function)
        self.dbw_enabled_bool = False
        self.dbw_enabled_sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_function)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # obtain min_speed for the yaw controller by adding the deceleration times time to the current velocity
        self.min_speed = 0 #max(0, decel_limit*time + self.current_velocity(needs to be finished))

        # TODO: Create `Controller` object
        # The Controller object returns the throttle and brake.
        self.controller = Controller(wheel_base, steer_ratio, self.min_speed, max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # self.loop()
        rospy.spin()
    
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo("Oncoming Waypoints are loading")
        self.base_waypoints = []
        for waypoint in waypoints.waypoints:
            # add to the waypoints list
            self.base_waypoints.append([waypoint.pose.pose.position.x, waypoint.pose.pose.position.y])
        self.base_waypoints = np.array(self.base_waypoints)
        rospy.loginfo("The number of oncoming waypoints are: " + str(self.base_waypoints.shape))

    def pose_cb(self, msg):
        rospy.loginfo("Position is updated")
        #Find the closest two waypoints given the position.
        self.steer = 0
        if self.prev_sample_time is None:
            self.sample_time = 0.02
            self.prev_sample_time = rospy.get_time()
        else:
            time = rospy.get_time()
            self.sample_time = time - self.prev_sample_time
            self.prev_sample_time = time
        if self.base_waypoints is not None:
            msg = (msg.pose.position.x, msg.pose.position.y)
            two_closest_points = self.base_waypoints[((self.base_waypoints-msg)**2).sum(axis=1).argsort()[:2]]
            self.cte = norm(np.cross(two_closest_points[0]-two_closest_points[1], two_closest_points[1]-msg))/norm(two_closest_points[0]-two_closest_points[1])
            pid_step = self.pid_controller.step(self.cte, self.sample_time)
            if self.dbw_enabled_bool:
                self.publish(throttle=.1, brake=0, steer=pid_step)


    def cte_function(self,msg):
        self.cte_bool = True
        self.cte =  msg.data
        rospy.loginfo("The CTE function has been activated: " + str(self.cte))
        pass
    
    def dbw_enabled_function(self,msg):
        self.dbw_enabled_bool =  msg.data
        self.dbw_enabled = msg
    
    def twist_cmd_function(self,msg):
        if self.cte==0:
            return
        # obtain linear velocity for yaw controller
        self.linear_velocity = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2 * 1.0)**(1.0/2)
        # obtain angular velocity for yaw controller
        self.angular_velocity = (msg.twist.angular.x**2 + msg.twist.angular.y**2 + msg.twist.angular.z**2 * 1.0)**(1.0/2)
        rospy.loginfo("Wanted linear velocity: " + str(self.linear_velocity))
        rospy.loginfo("Wanted angular velocity: " + str(self.angular_velocity))
        # rospy.loginfo("Current linear velocity: " + str(self.current_velocity))
        # rospy.loginfo("Current angular velocity: " + str(self.current_angular_velocity))
        #decide whether the angle is positive or negative
        self.steer_direction = 0
        if msg.twist.angular.z<0:
            self.steer_direction = 1
        if self.prev_sample_time is None:
            self.sample_time = 0.02
            self.prev_sample_time = rospy.get_time()
        else:
            time = rospy.get_time()
            self.sample_time = time - self.prev_sample_time
            self.prev_sample_time = time
        rospy.loginfo("The CTE from wpt_updtr: " + str(self.cte))
        rospy.loginfo("The sample time: " + str(self.sample_time))
        #publish in the twist function
        throttle, brake, steer = self.controller.control(self.min_speed, self.linear_velocity, self.angular_velocity, 
                                                                                self.current_velocity, self.current_angular_velocity, 
                                                                                self.steer_direction, self.cte, self.sample_time)
        pid_step = self.pid_controller.step(self.cte, self.sample_time)
        rospy.loginfo("The PID controller gives value of: " + str(pid_step))
        rospy.loginfo("The steering angle: " + str(steer))
        rospy.loginfo("The radius + PID controller gives value of: " + str(steer - pid_step))
        if self.dbw_enabled_bool:
            self.publish(throttle, brake, steer+pid_step)

    def current_velocity_function(self,msg):
        rospy.loginfo("Current velocity is loading")
        # obtain current_velocity for yaw controller
        self.current_velocity = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2 * 1.0)**(1.0/2)
        rospy.loginfo("The current velocity is: " + str(self.current_velocity))
        #obtain current_angular_velocity for controller
        self.current_angular_velocity = (msg.twist.angular.x**2 + msg.twist.angular.y**2 + msg.twist.angular.z**2 * 1.0)**(1.0/2)
        rospy.loginfo("The current angular velocity is: " + str(self.current_angular_velocity))
        # pass

    def loop(self):
        rate = rospy.Rate(5) # 50Hz
        while not rospy.is_shutdown():
            self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)
            self.twist_cmd_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_function)
            self.dbw_enabled_sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_function)
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            throttle, brake, steer = self.controller.control(self.min_speed, self.linear_velocity, self.angular_velocity, 
                                                                                    self.current_velocity, self.current_angular_velocity, 
                                                                                    self.steer_direction, self.cte, self.sample_time)
            if self.dbw_enabled_bool:
                self.publish(throttle, brake, steer)
            rate.sleep()

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

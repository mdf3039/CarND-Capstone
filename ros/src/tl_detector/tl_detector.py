#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        #Get the maximum velocity parameter
        self.maximum_velocity = self.kmph2mps(rospy.get_param('~velocity')) # change km/h to m/s

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        rospy.loginfo("The config type: " + str(type(self.config)))
        rospy.loginfo("The config sub type: " + str(type(self.config.keys)))
        rospy.loginfo("The config sub sub type: " + str(self.config))

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.current_pose = None
        self.base_waypoints = None
        self.lights = []

        self.stopping_waypoint_index = -1
        self.stopping_waypoint_distance = 1000

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.vehicle_traffic_lights_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.current_velocity = 0
        self.current_angular_velocity = 0
        self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_function)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub6_sub = rospy.Subscriber('/image_color', Image, self.image_cb)



        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.Unknown_Light = 4
        self.Green_Light = 2
        self.Yellow_Light = 1
        self.Red_Light = 0
        

        rospy.spin()

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def current_velocity_function(self,msg):
        # obtain current_velocity for yaw controller
        self.current_velocity = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2 * 1.0)**(1.0/2)
        #obtain current_angular_velocity for controller
        self.current_angular_velocity = (msg.twist.angular.x**2 + msg.twist.angular.y**2 + msg.twist.angular.z**2 * 1.0)**(1.0/2)
        pass

    def pose_cb(self, msg):
        #given the current position, find the closest traffic light stop line
        #and which waypoint is closest to that traffic light
        traffic_light_distances = []
        waypoint_distances = []
        #Transform all traffic light coordinates into the current_position coordinate space
        # create variables obtaining the placement of the vehicle
        cx_position = msg.pose.position.x
        cy_position = msg.pose.position.y
        cz_position = msg.pose.position.z
        cw_position = msg.pose.orientation.w
        self.current_pose = msg
        for each_stop_line in self.config['stop_line_positions']:
            #create variables for the placement of the traffic_light
            each_stop_linex = each_stop_line[0]
            each_stop_liney = each_stop_line[1]
            # transform the waypoint
            shift_x = each_stop_linex - cx_position
            shift_y = each_stop_liney - cy_position
            each_stop_linex = shift_x * math.cos(0-cw_position) - shift_y * math.sin(0-cw_position)
            each_stop_liney = shift_x * math.sin(0-cw_position) + shift_y * math.cos(0-cw_position)
            #append distance if x is positive, otherwise append a large number
            if each_stop_linex > 0:
                traffic_light_distances.append((each_stop_linex**2 + each_stop_liney**2*1.0)**(1.0/2))
            else:
                traffic_light_distances.append(100000000)
        #find the smallest distance to a traffic light
        nearest_light = np.amin(traffic_light_distances)
        #if the base waypoints are not loaded
        if self.base_waypoints is None:
            rospy.loginfo("THE BASE WAYPOINTS ARE NOT THERE")
            self.stopping_waypoint_index = 0
            self.stopping_waypoint_distance = 10000
            return
        #Transform all waypoint coordinates into the current position coordinate space
        for each_waypoint in self.base_waypoints.waypoints:
            #create variables for the placement of the waypoint
            each_waypointx = each_waypoint.pose.pose.position.x
            each_waypointy = each_waypoint.pose.pose.position.y
            # transform the waypoint
            shift_x = each_waypointx - cx_position
            shift_y = each_waypointy - cy_position
            each_waypointx = shift_x * math.cos(0-cw_position) - shift_y * math.sin(0-cw_position)
            each_waypointy = shift_x * math.sin(0-cw_position) + shift_y * math.cos(0-cw_position)
            wp_distance = (each_waypointx**2 + each_waypointy**2*1.0)**(1.0/2)
            #append the distance if x is positive and smaller than the nearest light's distance, otherwise append a small number
            if (each_waypointx > 0 and wp_distance < nearest_light):
                waypoint_distances.append(wp_distance)
            else:
                waypoint_distances.append(-1)
        #find the index of the largest distanced waypoint (which is the one closest to the nearest light)
        self.stopping_waypoint_index = np.argmax(waypoint_distances)
        self.stopping_waypoint_distance = np.amax(waypoint_distances)
        

    def waypoints_cb(self, msg):
        self.base_waypoints = msg

    def traffic_cb(self, msg):
        self.vehicle_traffic_lights = msg

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg
        #
        #
        # The msg or self.camera_image is the image. Feed it into your model. Return the traffic light 
        # classification as the variable state (not self.state) 
        # Unknown_Light = 4, Green_Light = 2, Yellow_Light = 1, Red_Light = 0
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        state = self.light_classifier.get_classification(cv_image) ##change this

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
        self.state_count += 1
        # implement the process traffic lights function
        self.process_traffic_lights()

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        stopping_waypoint_index = self.stopping_waypoint_index
        nearest_light = self.stopping_waypoint_distance
        # the result of the image_cb function is in the equation below
        traffic_light_value = self.last_state
        #obtain the minimum stopping distance possible given the acceleration, and jerk limits, and slow stop point
        acceleration_limit = 10.0 - 1.0
        slow_stop_point = 0
        min_stop_distance = .2*self.current_velocity + (self.current_velocity*(self.current_velocity-slow_stop_point)/acceleration_limit - acceleration_limit/2.0*((self.current_velocity-slow_stop_point)/acceleration_limit)**2) + (0.5*slow_stop_point**2)
        #obtain the maximum stopping distance by changing the acceleration limit to 6
        acceleration_limit -= 3.0
        max_stop_distance = .2*self.current_velocity + (self.current_velocity*(self.current_velocity-slow_stop_point)/acceleration_limit - acceleration_limit/2.0*((self.current_velocity-slow_stop_point)/acceleration_limit)**2) + (0.5*slow_stop_point**2)
        #If the velocity is less than 2*slow_stop_point and the distance to the light is less than 2*(0.5*slow_stop_point**2) and the light is red
        if (self.current_velocity<=2*slow_stop_point and nearest_light<=slow_stop_point**2 and traffic_light_value==self.Red_Light):
            None
        #if the distance to the nearest_light is more than the max_stop_distance, ignore it
        elif nearest_light > max_stop_distance:
            stopping_waypoint_index = -1
        # else if the traffic light is Yellow or Red and the distance to the nearest light is more than the min_stop_distance
        elif ((traffic_light_value==self.Red_Light or traffic_light_value==self.Yellow_Light) and nearest_light>min_stop_distance):
            None
        #else if the traffic light is unknown, stopping waypoint will be -2, telling whatever previous action to keep proceeding
        elif traffic_light_value==self.Unknown_Light:
            stopping_waypoint_index = -2
        #else if the traffic light is green, ignore it.
        elif traffic_light_value==self.Green_Light:
            stopping_waypoint_index = -1
        #publish the stopping waypoint index
        self.upcoming_red_light_pub.publish(stopping_waypoint_index)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

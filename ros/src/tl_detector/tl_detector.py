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
        # rospy.loginfo("The config type: " + str(type(self.config)))
        # rospy.loginfo("The config sub type: " + str(type(self.config.keys)))
        # rospy.loginfo("The config sub sub type: " + str(self.config))

        self.pose = None
        self.prev_pose = None
        self.waypoints = None
        self.camera_image = None
        self.current_pose = None
        self.base_waypoints = None
        self.lights = []
        self.c_image = None

        self.stopping_waypoint_index = -1
        self.stopping_waypoint_distance = 1000
        self.nearest_light_index = None

        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # self.vehicle_traffic_lights_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
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
        # sub6_sub = rospy.Subscriber('/image_color', Image, self.image_cb_function)



        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        # self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.Unknown_Light = 4
        self.Green_Light = 2
        self.Yellow_Light = 1
        self.Red_Light = 0
        
        self.loop_rate = 2
        # self.loop()
        # rospy.spin()

    def loop(self):
        rate = rospy.Rate(self.loop_rate) # 1Hz
        while not rospy.is_shutdown():
            self.actual_image_test(rospy.wait_for_message('/vehicle/traffic_lights', TrafficLightArray))
            # self.image_cb(rospy.wait_for_message('/image_color', Image))
            rate.sleep()

    def actual_image_test(self, msg):
        if self.nearest_light_index is None:
            return
        #using the actual sign of the traffic light instead of read from image
        state = int(msg.lights[self.nearest_light_index].state)
        # rospy.loginfo("Image Classified: " + str(state) + " at position " + str(msg.lights[self.nearest_light_index].pose.pose.position.x) + "," + str(msg.lights[self.nearest_light_index].pose.pose.position.y))
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
        self.state_count += 1
        # implement the process traffic lights function
        self.process_traffic_lights()
        # rospy.loginfo("Image Processed.")

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def current_velocity_function(self,msg):
        # obtain current_velocity for yaw controller
        self.current_velocity = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2 * 1.0)**(1.0/2)
        #obtain current_angular_velocity for controller
        self.current_angular_velocity = (msg.twist.angular.x**2 + msg.twist.angular.y**2 + msg.twist.angular.z**2 * 1.0)**(1.0/2)
        pass    

    def pose_cb(self, msg):
        self.pose = np.array([msg.pose.position.x,msg.pose.position.y])
        # given the current position, find the closest traffic light stop line
        if self.prev_pose is None or np.all(self.prev_pose == self.pose):
            self.prev_pose = self.pose - 0.1
        # find the distances from the current position and the stop lines
        stop_line_positions = np.array(self.config['stop_line_positions'])
        traffic_light_distances = np.sqrt(((stop_line_positions-self.pose)**2).sum(axis=1))
        # Find the sign of the dot product of the position vector and the traffic stop line vectors wrt the previous position
        dot_product_sign = np.sign(np.dot(stop_line_positions-self.prev_pose, self.pose-self.prev_pose))
        # Multiply the traffic light distances by their respective sign. Positive distances mean in front of car
        traffic_light_distances = np.multiply(traffic_light_distances,dot_product_sign)
        #find the smallest positive distance to a traffic light
        nearest_light = np.amin(traffic_light_distances[np.where(traffic_light_distances>=0)[0]])
        #obtain the index for the actual traffic sign image test
        self.nearest_light_index = np.where(traffic_light_distances==nearest_light)[0][0]
        if self.base_waypoints is None:
            # rospy.loginfo("THE BASE WAYPOINTS ARE NOT THERE")
            self.stopping_waypoint_index = 0
            self.stopping_waypoint_distance = 10000
            return
        # Do the same for the waypoints. find the distance from the current position and multiply it by the sign of the dot product
        base_waypoint_distances = np.sqrt(((self.base_waypoints-self.pose)**2).sum(axis=1))
        waypoint_dot_product_sign = np.sign(np.dot(self.base_waypoints-self.prev_pose, self.pose-self.prev_pose))
        base_waypoint_distances = np.multiply(base_waypoint_distances,waypoint_dot_product_sign)
        # Find the waypoint that is smaller than, but closest to, the nearest light distance
        self.stopping_waypoint_distance = np.amax(base_waypoint_distances[np.where(base_waypoint_distances<=nearest_light)[0]])
        self.stopping_waypoint_index  = np.where(base_waypoint_distances==self.stopping_waypoint_distance)[0][0]
        #the current position will be the previous position
        self.prev_pose = self.pose.copy()

    def waypoints_cb(self, msg):
        base_waypoints = []
        for each_waypoint in msg.waypoints:
            base_waypoints.append([each_waypoint.pose.pose.position.x, each_waypoint.pose.pose.position.y])
        self.base_waypoints = np.array(base_waypoints)

    def traffic_cb(self, msg):
        self.vehicle_traffic_lights = []
        for each_light in msg.lights:
            self.vehicle_traffic_lights.append([each_light.pose.pose.position.x, each_light.pose.pose.position.y])



    def image_cb_function(self, msg):
        rospy.loginfo("Image has arrived.")
        self.c_image = msg

    def image_cb(self, msg):
        if msg is None:
            rospy.loginfo("MSG is None.")
            return
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        rospy.loginfo("Image Obtained.")
        self.camera_image = msg
        #
        #
        # The msg or self.camera_image is the image. Feed it into your model. Return the traffic light 
        # classification as the variable state (not self.state) 
        # Unknown_Light = 4, Green_Light = 2, Yellow_Light = 1, Red_Light = 0
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        rospy.loginfo("Image Bridged.")
        state = self.light_classifier.get_classification(cv_image)
        rospy.loginfo("Image Classified.")

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
        rospy.loginfo("Image Processed.")

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
        #add on the current_velocity*rate to make sure it does not overlook the time gap
        max_stop_distance += self.current_velocity*1.0/self.loop_rate
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

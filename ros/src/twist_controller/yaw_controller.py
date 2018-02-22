from pid import PID
from math import atan

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, kp, ki, kd):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        self.pid_controller = PID(kp, ki, kd)


    def get_angle(self, radius, cte, sample_time):
        pid_step = self.pid_controller.step(cte, sample_time)
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        # rospy.loginfo("The radius gives value of: " + str(radius))
        # rospy.loginfo("The PID controller gives value of: " + str(pid_step))
        # rospy.loginfo("The radius + PID controller gives value of: " + str(radius + pid_step))
        # rospy.loginfo("The output angle is: " + str(angle))
        return max(self.min_angle, min(self.max_angle, angle - pid_step))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity, cte, sample_time):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity, cte, sample_time) if abs(angular_velocity) > 0. else 0.0;
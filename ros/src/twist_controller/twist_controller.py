from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius):
        # TODO: Implement
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        pass

    def control(self, min_speed, linear_velocity, angular_velocity, current_velocity, current_angular_velocity, steer_direction):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer_angle = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        if not steer_direction:
            steer_angle *= -1
        #If the current velocity is zero and the linear velocity is zero, keep the brakes on

        #If the desired velocity is larger than the current_velocity
        if linear_velocity >= current_velocity:
            #Use a throttle formula that represents the current velocity and desired velocity
            throttle = current_velocity*0.0113 + (linear_velocity-current_velocity)*1.0/40
            return throttle, 0, steer_angle
        elif current_velocity > linear_velocity:
            #Use the brake formula, mass*deceleration*wheel_radius
            brake = self.vehicle_mass * (current_velocity-linear_velocity)*self.wheel_radius
            return 0, brake, steer_angle

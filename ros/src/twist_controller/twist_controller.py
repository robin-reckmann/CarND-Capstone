import rospy
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs['vehicle_mass']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.update_rate = kwargs['update_rate']
        self.min_velocity = 1.0 # not sure how to set this, I assume when we are stopped we might want to steer still?
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_velocity,
                                            self.max_lat_accel, self.max_steer_angle)
    pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        target_velocity = args[0]
        target_yaw = args[1]
        current_velocity = args[2]
        enabled = args[3]
        steer = 0.0
        brake = 0.0
        throttle = 0.0

        if enabled:
            delta_velocity = target_velocity - current_velocity
            if delta_velocity > 0.0:
                # car should speed up
                if target_velocity > 1.0:
                    throttle = 5.0 + (50.0 * delta_velocity / target_velocity)
                else:
                    throttle = 1.0
            elif target_velocity < 1.0:
                brake = 100.0
            elif delta_velocity > -1.0:
                # do nothing if close to target speed
                pass
            else:
                brake = 10.0 + (90.0 * abs(delta_velocity) / current_velocity)

            # ask the yaw controller for the next steering command
            steer = self.yaw_controller.get_steering(target_velocity, target_yaw, current_velocity)
            if current_velocity < 4.0:
                if steer > 1.0:
                    steer = 1.0
                elif steer < -1.0:
                    steer = -1.0

        # Return throttle, brake, steer
        return throttle, brake, steer

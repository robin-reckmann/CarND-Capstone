import rospy
from yaw_controller import YawController
from pid import PID
from dbw_mkz_msgs.msg import BrakeCmd
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.vehicle_mass = kwargs['vehicle_mass']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.min_steer_angle = -1.0 * self.max_steer_angle
        self.update_rate = kwargs['update_rate']
        self.throttle_scale = kwargs['throttle_scale']

        self.min_velocity = 1.0 # not sure how to set this, I assume when we are stopped we might want to steer still?
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_velocity,
                                            self.max_lat_accel, self.max_steer_angle)
        self.low_pass_filter = LowPassFilter(0.2, #tau
                                             0.1) #ts 
        self.linear_pid = PID(0.8,  #kp
                              0.0008, #ki 
                              0.08, #kd
                              self.decel_limit, #mn
                              self.accel_limit) #mx

        self.cur_time = 0.0
        self.prev_time = 0.0


    def control(self, *args, **kwargs):
        target_velocity = args[0]
        target_yaw = args[1]
        current_velocity = args[2]
        enabled = args[3]
        steer = 0.0
        brake = 0.0
        throttle = 0.0


        self.curr_time = rospy.get_time()
        time_step = self.curr_time - self.prev_time
        self.prev_time = self.curr_time

        if enabled:
            delta_velocity = target_velocity - current_velocity
            linear_pid_update = self.linear_pid.step(delta_velocity, time_step)

            if delta_velocity > 0.0:
                # car should speed up
                throttle = linear_pid_update * self.throttle_scale
                brake = 0.0
            elif delta_velocity < -1 * (self.brake_deadband):
                # car should brake if target velocity is below brake deadband
                throttle = 0.0
                brake = abs(linear_pid_update) * self.vehicle_mass * self.wheel_radius 
            else: # brake only if target velocity is close to zero, otherwise keep values at 0
                brake = BrakeCmd.TORQUE_MAX if target_velocity < 0.1 and delta_velocity < 0.1 else 0.0
                throttle = 0.0


            # ask the yaw controller for the next steering command
            #filtered_yaw = self.low_pass_filter.filt(target_yaw)
            steer = self.yaw_controller.get_steering(target_velocity, target_yaw, current_velocity)

            rospy.loginfo("twist_controller: throttle=%s,brake=%s, steer=%s, delta_velocity=%s, pid_step=%s", throttle, brake, steer, delta_velocity, linear_pid_update)

        else: #not enabled
            self.linear_pid.reset()
        # Return throttle, brake, steer
        return throttle, brake, steer

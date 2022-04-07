#!/usr/bin/env python

# Available control modes:
# 0: Individually control prop speeds
# 1: Both prop speeds equal
# 2: Ramp up, according to a specified lower and upper speed limit and a given ramp up time
# If PI control is enabled, the force and torque measurements will be used to adapt the lower rotor (i.e. rotor 1)
# so that the total net torque is zero. This is done via a PI controller.
# This is only valid for modes 1 and 2 (i.e. when both props are controlled to run at the same speed or for ramp up)

import rospy
import numpy as np

from dynamic_reconfigure.server import Server
from thrust_test_controller_geranos.cfg import thrust_testConfig

from std_msgs.msg import UInt16 as cmd_msg
# from omav_hovery_msgs.msg import UAVStatus as cmd_msg
# from omav_hovery_msgs.msg import MotorStatus
# from mavros_msgs.msg import TiltrotorActuatorCommands as cmd_msg
from geometry_msgs.msg import WrenchStamped as wrench_msg


def rampFromTo(start,end,t,t_total):
    return t/t_total * (end-start) + start

class ControllerNode():
    def __init__(self):
        self.ramping = False
        # self.ft0Ready = False
        # self.ft1Ready = False
        self.control_mode = 0
        # self.t_err_int = 0
        # self.torque0 = np.zeros(3)
        # self.torque1 = np.zeros(3)
        # self.torque0Bias = np.zeros(3)
        # self.torque1Bias = np.zeros(3)
        # self.calibrated = False
        # self.torque0_stamp = rospy.get_rostime()
        # self.torque1_stamp = rospy.get_rostime()
        # self.ft_meas_t_avg_last = rospy.get_rostime().to_sec()
        srv = Server(thrust_testConfig, self.configCallback)
        topic = rospy.get_param('~topic', 'mavros/cmd')
        rospy.loginfo('topic = %s', topic)
        self.rate = rospy.Rate((rospy.get_param('~rate', 100)))

        msg = cmd_msg()
        pub = rospy.Publisher(topic, cmd_msg, queue_size=10)
        # rospy.Subscriber("/rokubimini/ft_sensor0/ft_sensor_readings/wrench",
        #                  wrench_msg, self.rokubiCallback0)
        # rospy.Subscriber("/rokubimini/ft_sensor1/ft_sensor_readings/wrench",
        #                  wrench_msg, self.rokubiCallback1)
        # Wait until sensors ready
        # while(not(self.ft0Ready and self.ft1Ready)):
        #     rospy.sleep(1)
        # Remove constant bias
        # self.removeBias()

        # rospy.Subscriber("/esc/telemetry", cmd_msg, self.telemetryCallback)

        while not rospy.is_shutdown():
            # print(self.torque0)
            # self.updatePIControl()
            if self.control_mode == 0:
                msg.data = self.config.speed
            elif self.control_mode == 1:
                msg = self.updateSmoothRamp()
            elif self.control_mode == 2:
                msg = self.updateStepRamp()
            # msg.header.stamp = rospy.get_rostime()
            pub.publish(msg)

            self.rate.sleep()

    def configCallback(self, config, level):
        if config.stop_motor == True:
            # config.control_mode = 0
            config.speed = 48
            config.start_ramp = False
        if self.ramping == False and (config.control_mode == 1 or config.control_mode == 2) and config.start_ramp == True:
            self.ramping = True
            self.ramp_start_time = rospy.get_rostime()
            print("Starting ramp.")
        self.control_mode = config.control_mode
        # if config.reset_integrator == True:
        #     self.t_err_int = 0
        if (config.control_mode != 1 and config.control_mode != 2) or config.start_ramp == False:
            self.ramping = False
        self.config = config
        return config

    def updateSmoothRamp(self):
        msg = cmd_msg()
        msg.data = 48
        if self.ramping == True:
            t_since_ramp_start = (rospy.get_rostime() - self.ramp_start_time).to_sec()
            if t_since_ramp_start <= self.config.ramp_time:
                speed_ref = rampFromTo(self.config.ramp_vel_start, self.config.ramp_vel_end, t_since_ramp_start, self.config.ramp_time)
                msg.data = int(speed_ref)
                # msg.u_rotors[1] = speed_ref# + self.config.pi_control * self.pi_correction
            elif t_since_ramp_start <= self.config.ramp_time + self.config.high_time:
                msg.data = self.config.ramp_vel_end
            else:
                self.config.start_ramp = False
                # self.ramping = False
                # params = { 'ramping' : 'False', 'my_int_parameter' : 5 }
                # config = client.update_configuration(params)
        return msg

    def updateStepRamp(self):
        msg = cmd_msg()
        ref = 48
        if self.ramping == True:
            t_since_ramp_start = (rospy.get_rostime() - self.ramp_start_time).to_sec()
            # Find current phase of ramp:
            config = self.config
            min_speed = config.min_speed
            max_speed = config.max_speed
            t_ramp = config.ramp_time_steps
            t_high = config.high_time
            t_idle = config.idle_time
            cycle_time = 2*t_ramp + t_high + t_idle
            n_cycle = int(t_since_ramp_start / cycle_time)
            # Cycle structure: | _______      |
            #                  |/       \_____|
            #                    t_high  t_idle
            if n_cycle >= config.n_steps:
                self.ramping = False
                config.start_ramp = False
            else:
                t_cycle = t_since_ramp_start - n_cycle * cycle_time
                ref_high = min_speed + (n_cycle+1) * (max_speed - min_speed) / (config.n_steps)
                if t_cycle <= t_ramp:
                    ref = rampFromTo(min_speed,ref_high,t_cycle,t_ramp)
                elif t_cycle <= t_ramp + t_high:
                    ref = ref_high
                elif t_cycle <= 2*t_ramp + t_high:
                    ref = rampFromTo(ref_high,min_speed,t_cycle-t_ramp-t_high,t_ramp)
        msg.data = int(ref)
        # msg.u_rotors[1] = ref
        return msg


    # def rokubiCallback0(self, data):
    #     self.ft0Ready = True
    #     self.torque0_stamp = data.header.stamp
    #     self.torque0 = np.array(
    #         [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
    #     if (self.calibrated):
    #         self.torque0 -= self.torque0Bias

    # def rokubiCallback1(self, data):
    #     self.ft1Ready = True
    #     self.torque1_stamp = data.header.stamp
    #     self.torque1 = np.array(
    #         [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
    #     if (self.calibrated):
    #         self.torque1 -= self.torque1Bias

    # def updatePIControl(self):
    #     torque_err = np.linalg.norm(self.torque0 - self.torque1)
    #     torque_err_dt = (self.torque0_stamp - self.torque1_stamp).to_sec()
    #     # Use average time between recent measurements to determine dt to previous measurements
    #     self.ft_meas_t_avg = 0.5 * \
    #         (self.torque0_stamp.to_sec() + self.torque1_stamp.to_sec())
    #     # Make sure that the two force/torque measurement timestamps are not too far apart
    #     if abs(torque_err_dt) > 0.1:
    #         print("Long time between f/t measurements: {}".format(torque_err_dt))
    #     # else:
    #     dt = self.ft_meas_t_avg - self.ft_meas_t_avg_last
    #     self.t_err_int = self.t_err_int + dt * torque_err
    #     self.ft_meas_t_avg_last = self.ft_meas_t_avg
    #     # Apply PI control law:
    #     self.pi_correction = self.config.pi_kp * \
    #         torque_err + self.config.pi_ki * self.t_err_int

    # def removeBias(self):
    #     iters = 0
    #     for i in range(1, 300):
    #         self.torque0Bias += self.torque0
    #         self.torque1Bias += self.torque1
    #         iters += 1
    #         rospy.sleep(1/self.rate)

    #     self.torque0Bias /= iters
    #     self.torque1Bias /= iters
    #     self.calibrated = True

    # def telemetryCallback(self, data):
    #     self.tlm_stamp = data.header.stamp
    #     erpm = data.rpm


if __name__ == "__main__":
    rospy.init_node("thrust_test_controller_node", anonymous=True)
    try:
        cn = ControllerNode()
    except rospy.ROSInterruptException:
        pass

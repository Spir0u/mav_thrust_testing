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

# from dynamic_reconfigure.server import Server
# from thrust_test_controller.cfg import thrust_testConfig

from std_msgs.msg import UInt16 as cmd_msg
# from omav_hovery_msgs.msg import UAVStatus as cmd_msg
# from omav_hovery_msgs.msg import MotorStatus
# from mavros_msgs.msg import TiltrotorActuatorCommands as cmd_msg
# from geometry_msgs.msg import WrenchStamped as wrench_msg


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
        self.rate = 100.0
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

        while not rospy.is_shutdown():
            # print(self.torque0)
            # self.updatePIControl()
            if self.control_mode == 0:
                msg.data = self.config.speed
            elif self.control_mode == 1:
                msg = self.updateSmoothRamp()
            elif self.control_mode == 3:
                msg = self.updateStepRamp()
            # msg.header.stamp = rospy.get_rostime()
            pub.publish(msg)
            if self.rate:
                rospy.sleep(1/self.rate)
            else:
                rospy.sleep(1.0)


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', msg.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('udoo_listener', anonymous=True)

    rospy.Subscriber('mavros/cmd', UInt16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

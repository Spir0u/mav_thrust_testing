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

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', msg.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('udoo_listener', anonymous=True)

    rospy.Subscriber('mavros/setpoint_raw/actuator_command', cmd_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

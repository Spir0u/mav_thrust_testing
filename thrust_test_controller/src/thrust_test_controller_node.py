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
from thrust_test_controller.cfg import thrust_testConfig

from mavros_msgs.msg import TiltrotorActuatorCommands as cmd_msg
from geometry_msgs.msg import WrenchStamped as wrench_msg

class ControllerNode():
	def __init__(self):
		self.ramping = False
		self.control_mode = 0
		self.t_err_int = 0
		self.torque0 = np.zeros(3)
		self.torque1 = np.zeros(3)
		self.torque0_stamp = rospy.get_rostime()
		self.torque1_stamp = rospy.get_rostime()
		self.ft_meas_t_avg_last = rospy.get_rostime().to_sec()
		srv = Server(thrust_testConfig, self.configCallback)
		topic = rospy.get_param('~topic', 'mavros/cmd')
		rospy.loginfo('topic = %s', topic)
		rate = 100.0
		msg = cmd_msg()
		print(msg)
		pub = rospy.Publisher(topic, cmd_msg, queue_size=10)
		rospy.Subscriber("ft0", wrench_msg, self.rokubiCallback0)
		rospy.Subscriber("ft1", wrench_msg, self.rokubiCallback1)
		while not rospy.is_shutdown():
			self.updatePIControl()
			if self.control_mode == 0:
				msg.u_rotors[0] = self.config.speed_upper
				msg.u_rotors[1] = self.config.speed_lower
			elif self.control_mode == 1:
				msg.u_rotors[0] = self.config.speed_both
				msg.u_rotors[1] = self.config.speed_both + self.config.pi_control * self.pi_correction
			elif self.control_mode == 2:
				if self.ramping == True:
					t_since_ramp_start = (rospy.get_rostime() - self.ramp_start_time).to_sec()
					if t_since_ramp_start <= self.config.ramp_time:
						speed_ref = t_since_ramp_start/self.config.ramp_time * (self.config.ramp_vel_end - self.config.ramp_vel_start) + self.config.ramp_vel_start
						msg.u_rotors[0] = speed_ref
						msg.u_rotors[1] = speed_ref + self.config.pi_control * self.pi_correction
			msg.header.stamp = rospy.get_rostime()
			pub.publish(msg)
			if rate:
			   rospy.sleep(1/rate)
			else:
			   rospy.sleep(1.0)
	def configCallback(self, config, level):
		if self.ramping == False and config.control_mode == 2 and config.start_ramp == True:
			self.ramping = True
			self.ramp_start_time = rospy.get_rostime()
		self.control_mode = config.control_mode
		if config.reset_integrator == True:
			self.t_err_int = 0
		if config.control_mode != 2 or config.start_ramp == False:
			self.ramping = False
		self.config = config
		return config
	def rokubiCallback0(self,data):
		self.torque0_stamp = data.header.stamp
		self.torque0 = np.array([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
	def rokubiCallback1(self,data):
		self.torque1_stamp = data.header.stamp
		self.torque1 = np.array([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
	def updatePIControl(self):
		torque_err = np.linalg.norm(self.torque0 - self.torque1)
		torque_err_dt = (self.torque0_stamp - self.torque1_stamp).to_sec()
		# Use average time between recent measurements to determine dt to previous measurements
		self.ft_meas_t_avg = 0.5 * (self.torque0_stamp.to_sec() + self.torque1_stamp.to_sec())
		# Make sure that the two force/torque measurement timestamps are not too far apart
		if abs(torque_err_dt) > 0.1:
			print("Long time between f/t measurements: {}".format(torque_err_dt))
		# else:
		dt = self.ft_meas_t_avg - self.ft_meas_t_avg_last
		self.t_err_int = self.t_err_int + dt * torque_err
		self.ft_meas_t_avg_last = self.ft_meas_t_avg
		# Apply PI control law:
		self.pi_correction = self.config.pi_kp * torque_err + self.config.pi_ki * self.t_err_int

if __name__ == "__main__":
	rospy.init_node("thrust_test_controller_node", anonymous = False)
	try:
		cn = ControllerNode()
	except rospy.ROSInterruptException: pass

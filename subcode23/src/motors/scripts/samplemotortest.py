#!/usr/bin/env python3

import rospy
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time

class MotorTest:
	def __init__(self):
	
		i2c_bus = busio.I2C(SCL, SDA)
		self.pca = PCA9685(i2c_bus)
		self.pca.frequency = 50
		
		self.motor_channels = {
			"back_left": 12,
			"left_vert": 11,
			"right_vert": 10,
			"back_right": 13,
			"front_right": 14,
			"front_left": 15
		}
		
	def set_motor(self, channel, pulse_width_us):
	
		pulse_length = 1000000
		pulse_length //= self.pca.frequency #50 Hz
		pulse_length //= 4096
		#duty_cycle = pulse_width_us * 1000 // pulse_length
		duty_cycle = int(pulse_width_us / pulse_length)
		if 0 <= duty_cycle <= 4095:
			self.pca.channels[channel].duty_cycle = duty_cycle
		else:
			rospy.logerr(f"Calculated duty cycle {duty_cycle} out of range for pulse width {pulse_width_us} us")
		
	def test_motor(self, channel):
		
		rospy.loginfo(f"Testing motor on channel {channel}")
		self.set_motor(channel, 1500)
		time.sleep(2)
		self.set_motor(channel, 1800)
		time.sleep(5)
		self.set_motor(channel, 1200)
		time.sleep(5)
		self.set_motor(channel, 1500)
		time.sleep(2)
		
	def run_tests(self):
	
		for motor, channel in self.motor_channels.items():
			self.test_motor(channel)
		self.pca.deinit()
		
if __name__ == '__main__':
	
	rospy.init_node('motor_test_node', anonymous=True)
	motor_test = MotorTest()
	motor_test.run_tests()
	rospy.loginfo("Motor testing completed.")
	

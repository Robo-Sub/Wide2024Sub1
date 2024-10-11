#!/usr/bin/env python3

import rospy
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from board import SCL, SDA
import busio
import time


class MotorTest:
	def __init__(self):
	
		i2c_bus = busio.I2C(SCL, SDA)
		self.pca = PCA9685(i2c_bus)
		self.pca.frequency = 50
		
		self.motor_channels = {
			"back_left": 3, #3
			"left_vert": 12, #12
			"right_vert": 2, #2
			"back_right": 0, #0
			"front_right": 4, #4
			"front_left": 1, #1
			"gripper": 6
		}
		
		self.motors = {name: servo.ContinuousServo(self.pca.channels[channel]) for name, channel in self.motor_channels.items()}
		
	def set_motor_throttle(self, motor_name, throttle):
		if motor_name in self.motors:
			self.motors[motor_name].throttle = throttle
		else:
			rospy.logerr(f"Invalid motor name")

	def initialize(self):
		rospy.loginfo("Initializing Thrusters")
		for motor_name in self.motors:
			self.set_motor_throttle(motor_name,0.0)
		time.sleep(.1)
		for motor_name in self.motors:
			self.set_motor_throttle(motor_name,0.05)
		time.sleep(.5)
		for motor_name in self.motors:
			self.set_motor_throttle(motor_name,0.0)
		rospy.loginfo("Initialization complete")
			
	# def set_motor(self, channel, pulse_width_us):
	
	# 	pulse_length = 1000000
	# 	pulse_length //= self.pca.frequency #50 Hz
	# 	pulse_length //= 4096
	# 	#duty_cycle = pulse_width_us * 1000 // pulse_length
	# 	duty_cycle = int(pulse_width_us / pulse_length)
	# 	if 0 <= duty_cycle <= 4095:
	# 		self.pca.channels[channel].duty_cycle = duty_cycle
	# 		rospy.loginfo(f"Duty cycle = {duty_cycle}, pulse width = {pulse_width_us} us")
	# 	else:
	# 		rospy.logerr(f"Calculated duty cycle {duty_cycle} out of range for pulse width {pulse_width_us} us")
		
	# def test_motor(self, channel):
		
	# 	rospy.loginfo(f"Testing motor on channel {channel}")
	# 	rospy.loginfo(f"Initializing @1500")
	# 	self.set_motor(channel, 1500)
	# 	time.sleep(1)
	# 	rospy.loginfo(f"Moving forward @1800")
	# 	self.set_motor(channel, 1800)
	# 	time.sleep(2)
	# 	rospy.loginfo(f"Moving backward @1200")
	# 	self.set_motor(channel, 1200)
	# 	time.sleep(2)
	# 	rospy.loginfo(f"Back to neutral @1500")
	# 	self.set_motor(channel, 1500)
	# 	time.sleep(1)

	def test_motor(self, motor_name):
		rospy.loginfo(f"Testing motor: {motor_name}")
		self.set_motor_throttle(motor_name, 0.05) #Neutral
		time.sleep(1)
		self.set_motor_throttle(motor_name, 0.15) #Forward
		time.sleep(1)
		self.set_motor_throttle(motor_name, 0.05) #Neutral
		time.sleep(2)
		self.set_motor_throttle(motor_name, -0.15) #Backwards
		time.sleep(2)
		self.set_motor_throttle(motor_name, 0.05) #Neutral
		time.sleep(1)

	def test_newesc(self,motor_name):                    #FOR NEW ESC
		rospy.loginfo(f"Testing motor: {motor_name}")
		self.set_motor_throttle(motor_name, 0.0) #Neutral
		time.sleep(1)
		self.set_motor_throttle(motor_name, 0.2) #Forward
		time.sleep(1)
		self.set_motor_throttle(motor_name, 0.0) #Neutral
		time.sleep(1)
		self.set_motor_throttle(motor_name, -0.2) #Backwards
		self.set_motor_throttle(motor_name, 0.0) #Neutral
		
	def set_gripper_position(self, position):
		if "gripper" in self.motors:
			self.motors["gripper"].throttle = position
		else:
			rospy.logerr("Gripper motor not initialized")

	def test_gripper(self):
		rospy.loginfo("Testing gripper")
		self.set_gripper_position(0.0) #Neutral (open)
		time.sleep(2)
		self.set_gripper_position(1.0) #Close
		time.sleep(2)
		self.set_gripper_position(0.0) #Neutral (open)
		time.sleep(2)

	def run(self):
		#self.initialize()
		while not rospy.is_shutdown():

			motor_name = input("Enter motor name to test (back_left, back_right, front_left, front_right, left_vert, right_vert, gripper) or 'exit' to quit: ")
			
			if motor_name == 'exit':
				#self.initialize()
				break
			if motor_name == "gripper":
				self.test_gripper()
			#if motor_name =="right_vert":
			#	self.test_newesc(motor_name)
			elif motor_name in self.motors:
				self.test_motor(motor_name)
			else:
				rospy.logerr("Invalid entry")
				
		self.pca.deinit()
		
if __name__ == '__main__':

	rospy.init_node('motor_test_node', anonymous=True)
	motor_test = MotorTest()
	motor_test.run()
	rospy.loginfo("Motor testing completed.")

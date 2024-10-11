#!/usr/bin/env python3

import rospy
from adafruit_servokit import ServoKit
import time

class MotorTest:
    def __init__(self):
        self.hat = ServoKit(channels=16)

        # Defining motors using ServoKit
        self.back_left = self.hat.continuous_servo[3]
        self.left_vert = self.hat.continuous_servo[4]
        self.right_vert = self.hat.continuous_servo[2]
        self.back_right = self.hat.continuous_servo[0]
        self.front_right = self.hat.continuous_servo[12]
        self.front_left = self.hat.continuous_servo[1]
        self.gripper = self.hat.continuous_servo[6]  # If you want to keep the gripper

        self.motors = {
            "back_left": self.back_left,
            "left_vert": self.left_vert,
            "right_vert": self.right_vert,
            "back_right": self.back_right,
            "front_right": self.front_right,
            "front_left": self.front_left,
            "gripper": self.gripper,
        }

    def set_motor_throttle(self, motor_name, throttle):
        if motor_name in self.motors:
            self.motors[motor_name].throttle = throttle
        else:
            rospy.logerr(f"Invalid motor name: {motor_name}")

    def initialize(self):
        rospy.loginfo("Initializing Thrusters")
        for motor_name in self.motors:
            self.set_motor_throttle(motor_name, 0.075)
        time.sleep(2)
        for motor_name in self.motors:
            self.set_motor_throttle(motor_name, 0.15)
        time.sleep(1)
        for motor_name in self.motors:
            self.set_motor_throttle(motor_name, 0.075)
        time.sleep(2)
        rospy.loginfo("Initialization complete")

    def test_motor(self, motor_name):
        rospy.loginfo(f"Testing motor: {motor_name}")
        self.set_motor_throttle(motor_name, 0.075)  # Neutral
        time.sleep(1)
        self.set_motor_throttle(motor_name, 0.15)  # Forward #.15
        time.sleep(1)
        self.set_motor_throttle(motor_name, 0.075)  # Neutral
        time.sleep(1)
        self.set_motor_throttle(motor_name, -0.15)  # Backwards #-.15
        time.sleep(1)
        self.set_motor_throttle(motor_name, 0.075)  # Neutral

    def set_gripper_position(self, position):
        if "gripper" in self.motors:
            self.motors["gripper"].throttle = position
        else:
            rospy.logerr("Gripper motor not initialized")

    def test_gripper(self):
        rospy.loginfo("Testing gripper")
        self.set_gripper_position(0.0)  # Neutral (open)
        time.sleep(2)
        self.set_gripper_position(1.0)  # Close
        time.sleep(2)
        self.set_gripper_position(0.0)  # Neutral (open)
        time.sleep(2)

    def run(self):
        self.initialize()
        while not rospy.is_shutdown():
            motor_name = input("Enter motor name to test (back_left, back_right, front_left, front_right, left_vert, right_vert, gripper) or 'exit' to quit: ")
            if motor_name == 'exit':
                break
            if motor_name == "gripper":
                self.test_gripper()
            elif motor_name in self.motors:
                self.test_motor(motor_name)
            else:
                rospy.logerr("Invalid entry")
                
        rospy.loginfo("Motor testing completed.")

if __name__ == '__main__':
    rospy.init_node('motor_test_node', anonymous=True)
    motor_test = MotorTest()
    motor_test.run()


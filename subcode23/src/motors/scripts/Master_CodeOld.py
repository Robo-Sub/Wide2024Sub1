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
            "back_left": 7,
            "left_vert": 8,
            "right_vert": 9,
            "back_right": 10,
            "front_right": 11,
            "front_left": 15,
            "gripper": 12
        }

        self.motors = {name: servo.ContinuousServo(self.pca.channels[channel]) for name, channel in self.motor_channels.items()}

    def set_motor_throttle(self, motor_name, throttle):
        if motor_name in self.motors:
            self.motors[motor_name].throttle = throttle
        else:
            rospy.logerr("Invalid motor name")

    def test_motor(self, motor_name):
        rospy.loginfo(f"Testing motor: {motor_name}")
        self.set_motor_throttle(motor_name, 0.05)  # Neutral
        time.sleep(1)
        self.set_motor_throttle(motor_name, 0.15)  # Forward
        time.sleep(2)
        self.set_motor_throttle(motor_name, 0.05) # Stop/Slow down
        time.sleep(1)
        self.set_motor_throttle(motor_name, -0.15)  # Backwards
        time.sleep(2)
        self.set_motor_throttle(motor_name, 0.05)  # Neutral
        time.sleep(1)

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

    def move_forward(self):
        rospy.loginfo("Moving forward")
        self.set_motor_throttle("back_left", 0.15)
        self.set_motor_throttle("back_right", 0.15)
        self.set_motor_throttle("front_left", 0.15)
        self.set_motor_throttle("front_right", 0.15)
        time.sleep(2)
        self.stop_all_motors()

    def move_backward(self):
        rospy.loginfo("Moving backward")
        self.set_motor_throttle("back_left", -0.15)
        self.set_motor_throttle("back_right", -0.15)
        self.set_motor_throttle("front_left", -0.15)
        self.set_motor_throttle("front_right", -0.15)
        time.sleep(2)
        self.stop_all_motors()

    def move_up(self):
        rospy.loginfo("Moving up")
        self.set_motor_throttle("left_vert", 0.15)
        self.set_motor_throttle("right_vert", 0.15)
        time.sleep(2)
        self.stop_all_motors()

    def move_down(self):
        rospy.loginfo("Moving down")
        self.set_motor_throttle("left_vert", -0.15)
        self.set_motor_throttle("right_vert", -0.15)
        time.sleep(2)
        self.stop_all_motors()

    def stop_all_motors(self):
        rospy.loginfo("Stopping all motors")
        for motor_name in self.motors.keys():
            self.set_motor_throttle(motor_name, 0.05)

    def run(self):
        while not rospy.is_shutdown():
            motor_name = input("Enter motor name to test (back_left, back_right, front_left, front_right, left_vert, right_vert, gripper), or 'forward', 'backward', 'up', 'down' to move the AUV, or 'exit' to quit: ")
            if motor_name == 'exit':
                break
            elif motor_name == "gripper":
                self.test_gripper()
            elif motor_name == "forward":
                self.move_forward()
            elif motor_name == "backward":
                self.move_backward()
            elif motor_name == "up":
                self.move_up()
            elif motor_name == "down":
                self.move_down()
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
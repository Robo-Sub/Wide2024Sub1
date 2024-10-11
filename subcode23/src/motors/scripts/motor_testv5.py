
#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 21:35:36 2022
@author: 18nlu, mknaw and mflar
"""

import cv2
import rospy
from std_msgs.msg import Int32
import numpy as np
import board
from adafruit_servokit import ServoKit
import time

# Initialize ROS node
rospy.init_node("color_motor_control")

# Initialize the motor kit
hat = ServoKit(channels=16)

# Define motors (Looking from the front)
back_left = hat.continuous_servo[12]
left_vert = hat.continuous_servo[11]
right_vert = hat.continuous_servo[10]
back_right = hat.continuous_servo[13]
front_right = hat.continuous_servo[14]
front_left = hat.continuous_servo[15]
thrusters = [back_left, left_vert, right_vert, back_right, front_right, front_left]

# Setting the pulse range for the motors
for i in thrusters:
    i.set_pulse_width_range(1100, 1900)

# Function to stop all motors
def stop_motors():
    for motor in thrusters:
        motor.throttle = 0.1
    rospy.loginfo("Motors stopped")

# Function to move all motors forward
def move_forward():
    for motor in thrusters:
        motor.throttle = 0.5  # Adjust the throttle as needed for your setup
    rospy.loginfo("Moving forward")

# Color publisher
color_pub = rospy.Publisher('camera_color', Int32, queue_size=10)

# Get parameter values
frequency = rospy.get_param('~/frequency', 2)

# Webcamera no 0 is used to capture the frames
cap = cv2.VideoCapture(0)

# Set variables
rate = rospy.Rate(frequency)
color = 0

# State variable to keep track of motor status
moving_forward = False

# Data collection loop
while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Failed to capture frame")
        continue

    # Create a downscaled frame
    size = frame.shape
    frame1 = np.zeros((int(size[0] / 10), int(size[1] / 10), size[2]), dtype=np.uint8)
    for i in range(0, int(size[0] / 10)):
        for j in range(0, int(size[1] / 10)):
            frame1[i, j] = frame[10 * i, 10 * j]

    # Define color BGR ranges
    color_ranges = {
        "red": ([9, 13, 112], [41, 35, 171], 1),
        "orange": ([7, 40, 136], [49, 104, 193], 2),
        "yellow": ([10, 100, 134], [46, 152, 186], 3),
        "green": ([95, 100, 29], [160, 181, 109], 4),
        "blue": ([90, 50, 20], [150, 120, 80], 5),
        "purple": ([80, 30, 40], [152, 84, 128], 6),
        "black": ([24, 19, 17], [70, 70, 60], 7)
    }

    # Calculate dem
    dem = 255 * frame1.shape[0] * frame1.shape[1]

    # Threshold for color detection
    thresh = 0.2

    max_reading = 0
    detected_color = None
    detected_color_value = None

    # Find color with the highest reading
    for color_name, (lower, upper, color_value) in color_ranges.items():
        mask = cv2.inRange(frame1, np.array(lower), np.array(upper))
        color_t = np.sum(mask) / dem
        rospy.loginfo(f'{color_name.capitalize()}: {color_t}')

        if color_t > max_reading:
            max_reading = color_t
            detected_color = color_name
            detected_color_value = color_value

    # Determine actions based on detected color
    if detected_color == "orange" and max_reading > thresh and not moving_forward:
        rospy.loginfo(f"Detected orange with reading {max_reading}")
        move_forward()
        moving_forward = True  # Update state to indicate moving forward
    elif detected_color == "purple" and max_reading > thresh and moving_forward:
        rospy.loginfo(f"Detected purple with reading {max_reading}")
        stop_motors()
        moving_forward = False  # Update state to indicate motors are stopped
    else:
        rospy.loginfo("No relevant color detected above the threshold")
        if moving_forward:
            # Keep moving forward if currently doing so
            rospy.loginfo("Continuing to move forward")

    # Publish detected color
    if detected_color_value is not None:
        color_pub.publish(detected_color_value)
    else:
        color_pub.publish(0)

    # Rotate frame and handle any interruptions
    frame1 = cv2.rotate(frame1, cv2.ROTATE_180)
    # cv2.imshow('frame1', frame1)

    # Break loop if the ESC key is pressed
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

    # Rate sleep
    rate.sleep()

# Cleanup
cv2.destroyAllWindows()
cap.release()


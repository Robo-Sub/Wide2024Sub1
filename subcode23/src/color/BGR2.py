#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 21:35:36 2022
@author: 18nlu, mknaw and mflar
"""
# Python program for Detection of a specific color using OpenCV with Python

import cv2
import rospy
from std_msgs.msg import Int32
import numpy as np

# Initialize ROS node
rospy.init_node("color_detection")  # Changed node name to follow ROS naming conventions

# Color publisher
color_pub = rospy.Publisher('camera_color', Int32, queue_size=10)

# Get parameter values
frequency = rospy.get_param('~/frequency', 2)

# Webcamera no 0 is used to capture the frames
cap = cv2.VideoCapture(0)

# Set variables
rate = rospy.Rate(frequency)
color = 0

# Data collection loop
while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Failed to capture frame")
        continue  # Skip the rest of the loop iteration if frame capture fails

    # Create new frame
    size = frame.shape
    frame1 = np.zeros((int(size[0] / 10), int(size[1] / 10), size[2]), dtype=np.uint8)
    size1 = frame1.shape

    # Pull every tenth pixel
    for i in range(0, int(size[0] / 10)):
        for j in range(0, int(size[1] / 10)):
            frame1[i, j] = frame[10 * i, 10 * j]

    # Color BGR definitions
    color_ranges = {
        "red": ([9, 13, 112], [41, 35, 171], 1),
        "orange": ([7, 40, 136], [49, 104, 193], 2),
        "yellow": ([10, 100, 134], [46, 152, 186], 3),
        "green": ([95, 100, 29], [160, 181, 109], 4),
        "blue": ([90, 50, 20], [150, 120, 80], 5),
        "purple": ([80, 30, 40], [152, 84, 128], 6),
        "black": ([24, 19, 17], [70, 70, 60], 7)
    }

    # Color masks and summation
    dem = 255 * size1[0] * size1[1]

    # Increased threshold
    thresh = 0.2

    max_reading = 0
    detected_color = None

    # Iterate through colors and find the color with the highest reading
    for color_name, (lower, upper, color_value) in color_ranges.items():
        mask = cv2.inRange(frame1, np.array(lower), np.array(upper))
        color_t = np.sum(mask) / dem
        rospy.loginfo(f'{color_name.capitalize()}: {color_t}')

        # Update max reading and detected color if the current reading is higher
        if color_t > max_reading:
            max_reading = color_t
            detected_color = color_name
            detected_color_value = color_value

    # Only report the detected color if it has a high enough reading
    if max_reading > thresh:
        print(f"Detected color: {detected_color} with reading {max_reading}")
        color_pub.publish(detected_color_value)
    else:
        print("No color detected above the threshold")
        color_pub.publish(0)

    # Rotate frame
    frame1 = cv2.rotate(frame1, cv2.ROTATE_180)

    # Display frame
    # cv2.imshow('frame1', frame1)

    # Handle keyboard interruption
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

    # Rate sleep
    rate.sleep()

# Cleanup
cv2.destroyAllWindows()
cap.release()


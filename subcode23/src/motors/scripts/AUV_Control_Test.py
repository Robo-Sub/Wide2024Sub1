#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit
import time

# Initialize the ServoKit for controlling thrusters
hat = ServoKit(channels=16)

# Defining motors (Looking from the front)
back_left = hat.continuous_servo[5]
left_vert = hat.continuous_servo[3]
right_vert = hat.continuous_servo[2]
back_right = hat.continuous_servo[4]
front_right = hat.continuous_servo[0]
front_left = hat.continuous_servo[1]
thrusters = [back_left, left_vert, right_vert, back_right, front_right, front_left]

# Setting the pulse range for the motors
for i in thrusters:
    i.set_pulse_width_range(1100, 1900)

# Initialize thrusters to stop
def initialize():
    print("Initializing")
    all_off()
    all_on(0)
    time.sleep(0.5)
    all_off()

def all_on(throttle):
    for i in thrusters:
        i.throttle = throttle
    time.sleep(0.5)

def all_off():
    print("Stopping")
    for i in thrusters:
        i.throttle = 0.1
    time.sleep(0.5)

def fwd_rev(throttle):
    print("Moving forward/backward")
    back_left.throttle = throttle
    back_right.throttle = throttle
    front_right.throttle = throttle
    front_left.throttle = throttle

def roll(throttle):
    print("Rolling")
    left_vert.throttle = -throttle
    right_vert.throttle = throttle

def yaw(throttle):
    print("Yawing")
    back_left.throttle = -throttle
    back_right.throttle = throttle
    front_right.throttle = throttle
    front_left.throttle = -throttle

def left_right(throttle):
    print("Strafing left/right")
    back_left.throttle = throttle
    back_right.throttle = -throttle
    front_right.throttle = throttle
    front_left.throttle = -throttle

def up_down(throttle):
    print("Rising/descending")
    left_vert.throttle = throttle
    right_vert.throttle = throttle

# Desired orientation (setpoint) in terms of pitch, roll, yaw
desired_orientation = Vector3(0.0, 0.0, 0.0)

# PID controller parameters
kp = 1.0
ki = 0.0
kd = 0.0

# Error accumulation for integral term
error_integral = Vector3(0.0, 0.0, 0.0)
previous_error = Vector3(0.0, 0.0, 0.0)

def imu_callback(data):
    global previous_error, error_integral

    # Extract current orientation from IMU data
    current_orientation = Vector3(
        data.angular_velocity.x,  # Roll
        data.angular_velocity.y,  # Pitch
        data.angular_velocity.z   # Yaw
    )

    # Calculate error (difference from desired orientation)
    error = Vector3(
        desired_orientation.x - current_orientation.x,
        desired_orientation.y - current_orientation.y,
        desired_orientation.z - current_orientation.z
    )

    # Update integral of error
    error_integral.x += error.x
    error_integral.y += error.y
    error_integral.z += error.z

    # Calculate derivative of error
    error_derivative = Vector3(
        error.x - previous_error.x,
        error.y - previous_error.y,
        error.z - previous_error.z
    )

    # PID control output
    control_output = Vector3(
        kp * error.x + ki * error_integral.x + kd * error_derivative.x,
        kp * error.y + ki * error_integral.y + kd * error_derivative.y,
        kp * error.z + ki * error_integral.z + kd * error_derivative.z
    )

    # Update previous error
    previous_error = error

    # Apply control to thrusters
    apply_control_to_thrusters(control_output)

def apply_control_to_thrusters(control_output):
    # Determine thruster commands based on control output
    # For simplicity, assuming that:
    # - control_output.z controls forward/reverse motion (fwd_rev)
    # - control_output.x controls yaw
    # - control_output.y controls vertical motion (up_down)
    
    fwd_rev(control_output.z)
    yaw(control_output.x)
    up_down(control_output.y)

def control_node():
    initialize()

    rospy.init_node('thruster_control_node', anonymous=True)

    # Subscribe to the IMU data topic
    rospy.Subscriber('/imu_out', Imu, imu_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass

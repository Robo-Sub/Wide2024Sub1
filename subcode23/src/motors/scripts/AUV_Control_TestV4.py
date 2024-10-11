#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from adafruit_servokit import ServoKit
import time

# Initialize the ServoKit for controlling thrusters
hat = ServoKit(channels=16)

# Defining motors (Looking from the front)
back_left = hat.continuous_servo[3]
left_vert = hat.continuous_servo[12]
right_vert = hat.continuous_servo[2]
back_right = hat.continuous_servo[0]
front_right = hat.continuous_servo[4]
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

def up_down(throttle):
    print("Rising/descending")
    left_vert.throttle = throttle
    right_vert.throttle = throttle

def stop(throttle):
    print("At Correct Depth")
    left_vert.throttle = throttle
    right_vert.throttle = throttle

# Desired depth in meters
desired_depth = 0.5  # Change this value to the desired depth
current_depth = 0.0

def imu_callback(data):
    pass  # Implement IMU handling logic if needed

def depth_callback(data):
    global current_depth
    current_depth = data.data
    rospy.loginfo(f"Received Depth: {current_depth:.2f} m")  # Display the current depth

def control_node():
    initialize()

    rospy.init_node('thruster_control_node', anonymous=True)

    # Subscribe to the IMU data topic
    rospy.Subscriber('/imu_out', Imu, imu_callback)
    rospy.Subscriber('/depthm', Float64, depth_callback)

    # Allow some time to ensure the AUV is in the pool
    rospy.sleep(20)

    curr_trust = 0

    rate = rospy.Rate(1)  # 1 Hz control loop
    while not rospy.is_shutdown():
        rospy.loginfo(f"Loop - Current Depth: {current_depth:.2f} m")  # Additional debug info in the loop
        # Ensure current_depth is updated via the depth_callback
        if current_depth < desired_depth - 0.1:  # If above the desired depth by more than 0.1 meters
            up_down(max(curr_trust - 0.02, -0.40))  # Move down
        elif current_depth > desired_depth + 0.1:  # If below the desired depth by more than 0.1 meters
            up_down(max(curr_trust + 0.02, 0.40))  # Move up
        
            

        rate.sleep()

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass

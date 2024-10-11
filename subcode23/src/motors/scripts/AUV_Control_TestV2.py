#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Accel, Vector3
from std_msgs.msg import Float64
from adafruit_servokit import ServoKit
import time

# Initialize the ServoKit for controlling thrusters
hat = ServoKit(channels=16)

# Defining motors (Looking from the front)
back_left = hat.continuous_servo[3]
left_vert = hat.continuous_servo[4]
right_vert = hat.continuous_servo[2]
back_right = hat.continuous_servo[0]
front_right = hat.continuous_servo[12]
front_left = hat.continuous_servo[1]
thrusters = [back_left, left_vert, right_vert, back_right, front_right, front_left]

#gobal orient vector
current_orientation = Vector3(0,0,0)

# Setting the pulse range for the motors
for i in thrusters:
    i.set_pulse_width_range(1100, 1900)

# Initialize thrusters to stop
def initialize():
    print("Initializing")
    all_off()
    all_on(0.05)
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

def up_downL(throttle):
    print("Rising/descending")
    left_vert.throttle = throttle
    #right_vert.throttle = -throttle

def up_downR(throttle):
    right_vert.throttle = throttle
    
def stopL(throttle):
    print("At Correct Depth")
    left_vert.throttle = throttle
   #right_vert.throttle = -throttle

def stopR(throttle):
    right_vert.throttle = throttle

def move_forwardFL(throttle):                       #CONTROL FRONT LEFT
    if y1 > -0.01 or y1 < 0.01:
        front_left.throttle = throttle
    elif y1 > 0.02:                             #TURN DOWN FL AND BL
        front_left.throttle = (throttle*(3/4)) 
    else:                                       
        front_left.throttle = throttle

def move_forwardFR(throttle):                       #CONTROL FRONT RIGHT
    if y1 > -0.01 or y1 < 0.01:
        front_right.throttle = throttle
    elif y1 < -0.02:
        front_right.throttle = (throttle*(3/4))
    else:
        front_right.throttle = throttle


def move_forwardBL(throttle):                       #CONTROL BACK LEFT
    if y1 > -0.01 or y1 < 0.01:
        back_left.throttle = throttle
    elif y1 > 0.02:
        back_left.throttle = (throttle*(3/4))
    else:
        back_left.throttle = throttle

def move_forwardBR(throttle):                       #CONTROL BACK RIGHT
    if y1 > -0.01 or y1 < 0.01:
        back_right.throttle = throttle
    elif y1 < -0.02:
        back_right.throttle = (throttle*(3/4))
    else:
        back_right.throttle = throttle

# Desired depth in meters
desired_depth = 0.75  # Change this value to the desired depth
current_depth = 0.0
l = 0

def imu_callback(data):

    current_orientation = Vector3(
    data.linear.x,  # Roll
    data.linear.y,  # Pitch
    data.linear.z   # Yaw #OR data.angular.z
    )

def depth_callback(data):
    global current_depth
    current_depth = data.data
    rospy.loginfo(f"Received Depth: {current_depth:.2f} m")  # Display the current depth

def control_node():
    global l, x1, y1, z1
    initialize()

    rospy.init_node('thruster_control_node', anonymous=True)

    # Subscribe to the IMU data topic
    rospy.Subscriber('/imu_out', Accel, imu_callback)
    rospy.Subscriber('/depthm', Float64, depth_callback)

    # Allow some time to ensure the AUV is in the pool
    rospy.sleep(20)

    x1 = 0
    y1 = 0
    z1 = 0

    rate = rospy.Rate(1)  # 1 Hz control loop
    while not rospy.is_shutdown():
        x1 = round(current_orientation.y,2)
        y1 = round(current_orientation.x,2)
        z1 = round(current_orientation.z,2)
        
        rospy.loginfo(f"Loop - Current Depth: {current_depth:.2f} m")  # Additional debug info in the loop
        # Ensure current_depth is updated via the depth_callback
        if current_depth < desired_depth - 0.01:  # If above the desired depth by more than 0.1 meters
            up_downL(0.65)  # Move down #T1:-0.4B T2:-0.5G-P T3:-0.55G-B? T4:-0.6B T5:-0.65B T6: -0.75
            up_downR(0.85)               #T1:0.7B T2:0.7G-P T3:0.65G-B? T4:0.7B T5:0.75B T6:0.85

        elif current_depth > desired_depth + 0.01:  # If below the desired depth by more than 0.1 meters
            up_downL(0.45)  # Move up   #T1: -0.3 #B T2: -0.2G T3: -0.15 T4: -0.3
            up_downR(0.65)               #T2: 0.5 #G T2: 0.5G T3: 0.45 T4:0.45
            l = 1
        #else:
        #    stopL(0.45)  # Maintain current depth
        #    stopR(0.55)
        #    l = 1
        if l == 1:
            move_forwardFL(-0.5)
            move_forwardFR(-0.55)
            move_forwardBL(-0.55)
            move_forwardBR(-0.55)

        rate.sleep()

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass

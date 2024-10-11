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

# Global orientation vector and desired orientation
current_orientation = Vector3(0, 0, 0)
desired_orientation = Vector3(0.0, 0.5, 0.0)  # Example desired orientation (Pitch, Roll, Yaw)

# Setting the pulse range for the motors
for thruster in thrusters:
    thruster.set_pulse_width_range(1100, 1900)

# Initialize thrusters to stop
def initialize():
    print("Initializing")
    all_off()
    all_on(0.05)
    time.sleep(0.5)
    all_off()

def all_on(throttle):
    for thruster in thrusters:
        thruster.throttle = throttle
    time.sleep(0.5)

def all_off():
    print("Stopping")
    for thruster in thrusters:
        thruster.throttle = 0.1
    time.sleep(0.5)

def up_downL(throttle):
    print("Rising/descending")
    left_vert.throttle = throttle

def up_downR(throttle):
    right_vert.throttle = throttle
    
def stopL(throttle):
    print("At Correct Depth")
    left_vert.throttle = throttle

def stopR(throttle):
    right_vert.throttle = throttle

# Simplified move_forward functions
def move_forwardFL(throttle):  # CONTROL FRONT LEFT
    front_left.throttle = throttle

def move_forwardFR(throttle):  # CONTROL FRONT RIGHT
    front_right.throttle = throttle

def move_forwardBL(throttle):  # CONTROL BACK LEFT
    back_left.throttle = throttle

def move_forwardBR(throttle):  # CONTROL BACK RIGHT
    back_right.throttle = throttle

# Desired depth in meters
desired_depth = 0.55  # Change this value to the desired depth
current_depth = 0.0
l = 0

def imu_callback(data):
    global current_orientation
    # Capture angular values for orientation (pitch, roll, yaw)
    current_orientation = Vector3(
        data.angular.x,  # Pitch
        data.angular.y,  # Roll
        data.angular.z   # Yaw
    )

def depth_callback(data):
    global current_depth
    current_depth = data.data
    rospy.loginfo(f"Received Depth: {current_depth:.2f} m")  # Display the current depth

def adjust_orientation():
    global desired_orientation, current_orientation

    if desired_orientation is None:
        rospy.logwarn("Desired orientation is not set!")
        return

    # Calculate the difference between the current and desired orientation
    delta_x = current_orientation.x - desired_orientation.x  # Pitch difference
    delta_y = current_orientation.y - desired_orientation.y  # Roll difference
    delta_z = current_orientation.z - desired_orientation.z  # Yaw difference

    # Adjust thrusters based on the roll difference (delta_y)
    if delta_x > 0.01:  # Example threshold for correcting roll deviation to the left
        move_forwardFR(-0.45)  # Reduce right side thrust to correct leftward deviation
        move_forwardBR(-0.45)
        move_forwardFL(-0.55)
        move_forwardBL(-0.55)
    elif delta_x < -0.01:  # Example threshold for correcting roll deviation to the right
        move_forwardFR(-0.55)  # Reduce left side thrust to correct rightward deviation
        move_forwardBR(-0.55)
        move_forwardFL(-0.45)
        move_forwardBL(-0.45)
    if delta_z > 0.01:
        up_downL(0.45)
        up_downR(0.65)
    elif delta_z < -0.01:
        up_downL(0.65)
        up_downR(0.85)

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
        x1 = round(current_orientation.x, 2)
        y1 = round(current_orientation.y, 2)
        z1 = round(current_orientation.z, 2)
        
        rospy.loginfo(f"Loop - Current Depth: {current_depth:.2f} m")  # Additional debug info in the loop

        # Ensure current_depth is updated via the depth_callback
        #if l == 0:
        if current_depth < desired_depth - 0.01:  # If above the desired depth by more than 0.1 meters
            up_downL(0.65)
            up_downR(0.85)

        elif current_depth > desired_depth + 0.01:  # If below the desired depth by more than 0.1 meters
            up_downL(0.25)
            up_downR(0.45)
            l = 1

        if l == 1:
            #move_forwardFL(-0.5)
            #move_forwardFR(-0.5)
            move_forwardBL(-0.4)
            move_forwardBR(-0.5)
            #adjust_orientation()  # Call the function to adjust orientation based on IMU data

        rate.sleep()

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass

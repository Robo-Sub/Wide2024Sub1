import rospy
import numpy as np
from std_msgs.msg import String, Float64
from adafruit_servokit import ServoKit


# Initialize ROS node
rospy.init_node("motor_control")

# Initialize the motor kit
hat = ServoKit(channels=16, frequency = 100)

desired_depth = 0.5

# Define motors (Looking from the front)
back_left = hat.continuous_servo[3]
left_vert = hat.continuous_servo[12]
right_vert = hat.continuous_servo[2]
back_right = hat.continuous_servo[0]
front_right = hat.continuous_servo[4]
front_left = hat.continuous_servo[1]
thrusters = [back_left, left_vert, right_vert, back_right, front_right, front_left]
planar_thrusters = [back_left, back_right, front_right, front_left]

def initialize():
	print("Initializing")
	stop_motors()
	all_on(0)
	rospy.sleep(0.5)
	stop_motors()

# Set pulse range for the motors
for motor in thrusters:
    motor.set_pulse_width_range(1100, 1900)
right_vert.set_pulse_width_range(1000, 2000)


# Function to stop all motors
def stop_motors():
    for motor in thrusters:
        motor.throttle = 0.1
    rospy.loginfo("Motors stopped")

def all_on(throttle):
	for i in thrusters:
		i.throttle = throttle
	rospy.sleep(0.5)

# Function to move all motors forward
def move_forward():
    for motor in planar_thrusters:
        motor.throttle = 0.4  # Adjust the throttle as needed for your setup
    rospy.loginfo("Moving forward")

# Function to increase vertical thruster speeds
def increase_vertical_thrust():
    step = 0.01  # Step size for increasing speed
    left_vert.throttle = min(left_vert.throttle + step, 0.4)  # Limit to a maximum throttle of 1.0
    right_vert.throttle = min(right_vert.throttle + step, 0.4)
    rospy.loginfo(f"Increasing vertical thrust: left_vert={left_vert.throttle}, right_vert={right_vert.throttle}")

# Function to decrease vertical thruster speeds
def decrease_vertical_thrust():
    step = 0.01  # Step size for decreasing speed
    left_vert.throttle = max(left_vert.throttle - step, -0.40)  # Limit to a minimum throttle of -1.0
    right_vert.throttle = max(right_vert.throttle - step, -0.40)
    rospy.loginfo(f"Decreasing vertical thrust: left_vert={left_vert.throttle}, right_vert={right_vert.throttle}")

# Function to step up motor speed by 10
def increase_thrust_by_10():
    step = 0.01  # Step size for increasing speed
    for motor in planar_thrusters:
        motor.throttle = min(motor.throttle + step, 0.4)
    rospy.loginfo(f"Increasing thrust by 10: {[(motor.throttle) for motor in planar_thrusters]}")

# Function to step down motor speed by 10
def decrease_thrust_by_10():
    step = 0.01  # Step size for decreasing speed
    for motor in planar_thrusters:
        motor.throttle = max(motor.throttle - step, -0.4)
    rospy.loginfo(f"Decreasing thrust by 10: {[(motor.throttle) for motor in planar_thrusters]}")

# Determine actions based on detected color    
def pick_move_c(color):
    if color == "orange":
        # Increase motor speed by 10 when orange is detected
        increase_thrust_by_10()
    elif color == "purple":
        # Decrease motor speed by 10 when purple is detected
        decrease_thrust_by_10()

def set_depth(actual):
    
    if (np.isclose(round(actual,1), desired_depth, 0.1)):
        pass
    elif (round(actual,1) < desired_depth):
        increase_vertical_thrust()
    else:
        decrease_vertical_thrust()
    

    
# Call the initialization function at the start of the program
initialize()
rospy.sleep(20)

#set_depth
rospy.Subscriber("depth",Float64, set_depth)

rospy.sleep(10)
#move_forward(5)
#rospy.Subscriber("camera_color", String, pick_move_c)

rospy.signal_shutdown("end of test")


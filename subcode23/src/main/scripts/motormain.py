import rospy
from std_msgs.msg import Int32
import numpy as np
from adafruit_servokit import ServoKit
import openCV as cv2

# Initialize ROS node
rospy.init_node("color_motor_control")

# Initialize the motor kit
hat = ServoKit(channels=16)

class MotorControl:
    '''Could put docstring here'''
    # Define motors (Looking from the front)
    back_left = hat.continuous_servo[12]
    left_vert = hat.continuous_servo[11]
    right_vert = hat.continuous_servo[10]
    back_right = hat.continuous_servo[13]
    front_right = hat.continuous_servo[14]
    front_left = hat.continuous_servo[15]
    thrusters = [back_left, left_vert, right_vert, back_right, front_right, front_left]
    planar_thrusters = [back_left, back_right, front_right, front_left]

    for motor in thrusters:
        '''Set pulse range for the motors'''
        motor.set_pulse_width_range(1100, 1900)

    def stop_motors(self):
        '''Stop all motors'''
        for motor in self.thrusters:
            motor.throttle = 0.1
        rospy.loginfo("Motors stopped")

    def move_forward(self):
        '''Set all planar motors to forward'''
        for motor in self.planar_thrusters:
            motor.throttle = 0.5  # Adjust the throttle as needed for your setup
        rospy.loginfo("Moving forward")

    def increase_vertical_thrust(self):
        '''Increases vertical thruster speeds'''
        step = 0.1  # Step size for increasing speed
        left_vert.throttle = min(left_vert.throttle + step, 1.0)  # Limit to a maximum throttle of 1.0
        right_vert.throttle = min(right_vert.throttle + step, 1.0)
        rospy.loginfo(f"Increasing vertical thrust: left_vert={left_vert.throttle}, right_vert={right_vert.throttle}")

    def decrease_vertical_thrust(self):
        '''Function to decrease vertical thruster speeds'''
        step = 0.1  # Step size for decreasing speed
        self.left_vert.throttle = max(self.left_vert.throttle - step, -1.0)  # Limit to a minimum throttle of -1.0
        self.right_vert.throttle = max(self.right_vert.throttle - step, -1.0)
        rospy.loginfo(f"Decreasing vertical thrust: left_vert={self.left_vert.throttle}, right_vert={self.right_vert.throttle}")

    def increase_thrust_by_10(self):
        '''Function to step up motor speed by 10'''
        step = 0.1  # Step size for increasing speed
        for motor in self.thrusters:
            motor.throttle = min(motor.throttle + step, 1.0)
        rospy.loginfo(f"Increasing thrust by 10: {[(motor.throttle) for motor in self.thrusters]}")

    def decrease_thrust_by_10(self):
        '''Function to step down motor speed by 10'''
        step = 0.1  # Step size for decreasing speed
        for motor in self.thrusters:
            motor.throttle = max(motor.throttle - step, -1.0)
        rospy.loginfo(f"Decreasing thrust by 10: {[(motor.throttle) for motor in self.thrusters]}")

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

# Call the initialization function at the start of the program
submarine = MotorControl()
submarine.stop_motors()



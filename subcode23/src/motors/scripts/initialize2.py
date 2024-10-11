#!/usr/bin/env python3

import board
import time
import busio
import adafruit_pca9685

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

# Set frequency (consider experimenting with different frequencies)
hat.frequency = 250  # Example: 60 Hz

# Define motors
motor_forward_left = hat.channels[1]
motor_forward_right = hat.channels[0]
motor_up_left = hat.channels[3]
motor_up_right = hat.channels[2]
motor_side_front = hat.channels[4]
motor_side_back = hat.channels[5]

# Define total duty cycle range
total = 0xFFFF

# Initialize motors to a neutral state
def initialize_motors():
    neutral_duty_cycle = int(total * 0.5)
    neutral_duty_cycle2 = int(total * 0.4)
    motor_forward_left.duty_cycle = neutral_duty_cycle
    motor_forward_right.duty_cycle = neutral_duty_cycle
    motor_up_left.duty_cycle = neutral_duty_cycle
    motor_up_right.duty_cycle = neutral_duty_cycle2
    motor_side_front.duty_cycle = neutral_duty_cycle
    motor_side_back.duty_cycle = neutral_duty_cycle
    print("Motors initialized to neutral state.")

# Main initialization function
def main():
    time.sleep(5)
    print("Initializing motors...")
    initialize_motors()
    time.sleep(5)
    print("Initialization complete.")

if __name__ == '__main__':
    main()


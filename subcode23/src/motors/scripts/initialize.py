#!/usr/bin/env python3
import board
import time
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)

hat = adafruit_pca9685.PCA9685(i2c)
x=16
hat.frequency = int(60*0.9)

motor_forward_left = hat.channels[14]
motor_forward_right = hat.channels[15]
motor_up_left = hat.channels[10]
motor_up_right = hat.channels[11]
motor_side_front = hat.channels[13]
motor_side_back = hat.channels[12]
total = 0xffff

time.sleep(5)
print('Initializing')

#Running the motors (Not really giving them the initialization command
motor_forward_left.duty_cycle = int(total*15/40)
motor_forward_right.duty_cycle = int(total*15/40)

motor_up_left.duty_cycle = int(total*15/40)
motor_up_right.duty_cycle = int(total*15/40)

motor_side_front.duty_cycle = int(total*15/40)
motor_side_back.duty_cycle = int(total*15/40)
time.sleep(5)
print('Initialization Complete')

# while True:
#     print('Drop')
#     motor_up_left.duty_cycle = int(total*x/40)  #unsure if +PWM will go up or down
#     motor_up_right.duty_cycle = int(total*x/40)
#     time.sleep(5)
#     motor_up_left.duty_cycle = int(total*15/40)
#     motor_up_right.duty_cycle = int(total*15/40)
#     time.sleep(1)

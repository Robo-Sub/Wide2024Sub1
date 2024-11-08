#!/usr/bin/env python3
#import rospy
import board
import time
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)

hat = adafruit_pca9685.PCA9685(i2c)
freq = 54
hat.frequency = freq # may need to check this
#hat.set_pulse_width_range(1100,1900)
#hat.frequency = 110

#hat channels subject to change
motor_front_left = hat.channels[14]
motor_front_right = hat.channels[15]
motor_up_left = hat.channels[10]
motor_up_right = hat.channels[11]
motor_rear_left = hat.channels[13]
motor_rear_right = hat.channels[12]

#hat.set_pulse_width_range(1100,1900)

#Changing the range
total = 0xffff

#time.sleep(7)

#def calc_cycle(throttle):  #throttle = %thrust form -100 to 100
#    if throttle != 0:
#        cycle = hex(round((throttle + 100) / 200 * 0xffff))
#    else:
#        cycle = hex(0)
#    print(cycle)
#    return int(cycle,0)

#def calc_cycle(throttle):
#	if throttle == 0:
#		cycle = 0x7fff
#	else:
#		cycle = int(((throttle + 100) / 200) * 65535)
#	return cycle

def calc_cycle(throttle): #throttle is in microseconds
	period = 1/freq
	duty = throttle/period
	return duty

def all_off():
#    rospy.loginfo('All motors off')
    print('All motors off\n')
    throttle = 0
    motor_front_left.duty_cycle = calc_cycle(throttle)
    motor_front_right.duty_cycle = calc_cycle(throttle)
    motor_rear_left.duty_cycle = calc_cycle(throttle)
    motor_rear_right.duty_cycle = calc_cycle(throttle)
    time.sleep(0.2)


def fwd_thrust(dur_sec, throttle):
#    rospy.loginfo('Forward thrust')
    print('Forward thrust\n')
    while (dur_sec > 0):
        motor_front_left.duty_cycle = calc_cycle(throttle)
        motor_front_right.duty_cycle = calc_cycle(throttle)
        motor_rear_left.duty_cycle = calc_cycle(throttle)
        motor_rear_right.duty_cycle = calc_cycle(throttle)
        dur_sec = dur_sec - 0.02
        time.sleep(0.02)
    all_off()

def rev_thrust(dur_sec, throttle):
#    rospy.loginfo('Reverse thrust')
    print('Reverse thrust\n')
    while (dur_sec > 0):
        motor_front_left.duty_cycle = calc_cycle(-throttle)
        motor_front_right.duty_cycle = calc_cycle(-throttle)
        motor_rear_left.duty_cycle = calc_cycle(-throttle)
        motor_rear_right.duty_cycle = calc_cycle(-throttle)
        dur_sec = dur_sec - 0.02
        time.sleep(0.02)
    all_off()

def cw_yaw(dur_sec, throttle):
#    rospy.loginfo('Clocwise yaw')
    print('Clockwise yaw\n')
    while (dur_sec > 0):
        motor_front_left.duty_cycle = calc_cycle(throttle)
        motor_front_right.duty_cycle = calc_cycle(-throttle)
        motor_rear_left.duty_cycle = calc_cycle(throttle)
        motor_rear_right.duty_cycle = calc_cycle(-throttle)
        dur_sec = dur_sec - 0.02
        time.sleep(0.02)
    all_off()

def ccw_yaw(dur_sec, throttle):
#    rospy.loginfo('Counterclocwise yaw')
    print('Counterclockwise yaw\n')
    while (dur_sec > 0):
        motor_front_left.duty_cycle = calc_cycle(-throttle)
        motor_front_right.duty_cycle = calc_cycle(throttle)
        motor_rear_left.duty_cycle = calc_cycle(-throttle)
        motor_rear_right.duty_cycle = calc_cycle(throttle)
        dur_sec = dur_sec - 0.02
        time.sleep(0.02)
    all_off()

if __name__ == "__main__":

    #initializes the motors
    all_off()
    fwd_thrust(2,5)

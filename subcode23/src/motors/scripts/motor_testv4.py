import board
import time
from adafruit_servokit import ServoKit

#Constant
hat = ServoKit(channels=16)

#Defining my motors (Looking from the front)
back_left = hat.continuous_servo[5]
left_vert = hat.continuous_servo[3]
right_vert = hat.continuous_servo[2]
back_right = hat.continuous_servo[4]
front_right = hat.continuous_servo[0]
front_left = hat.continuous_servo[1]
thrusters = [back_left, left_vert, right_vert, back_right,front_right,front_left]

#Setting the pulse range for the motors
for i in thrusters:
	i.set_pulse_width_range(1100,1900)

#Initializing the motors (For whatever reason 0.1 is stop)
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
	print("Moving")
	back_left.throttle = throttle
	back_right.throttle = throttle
	front_right.throttle = throttle
	front_left.throttle = throttle
	time.sleep(0.5)

def roll(throttle):
	print("Rolling")
	left_vert.throttle = -throttle
	right_vert.throttle = throttle
	time.sleep(0.5)

def yaw(throttle):
	print("Yawing")
	back_left.throttle = -throttle
	back_right.throttle = throttle
	front_right.throttle = throttle
	front_left.throttle = -throttle
	time.sleep(0.5)


def left_right(throttle):
	print("Strafing")
	back_left.throttle = throttle
	back_right.throttle = -throttle
	front_right.throttle = throttle
	front_left.throttle = -throttle
	time.sleep(0.5)

def up_down(throttle):
	print("Rising")
	left_vert.throttle = throttle
	right_vert.throttle = throttle
	time.sleep(0.5)

#Movement Parameters (Throttle goes from -1 to 1, 0.1 is stop)

#Testing (0 is 10% in some direction)
initialize()
time.sleep(30)

all_on(0.3)
time.sleep(5)
all_off()

test_throttle = -1

fwd_rev(test_throttle)
time.sleep(30)
all_off()

fwd_rev(-test_throttle)
time.sleep(30)
all_off()

left_right(-test_throttle)
time.sleep(5)
all_off()

left_right(test_throttle)
time.sleep(5)
all_off()

roll(-test_throttle)
time.sleep(5)
all_off()

roll(test_throttle)
time.sleep(5)
all_off()

yaw(-test_throttle)
time.sleep(5)
all_off()

yaw(test_throttle)
time.sleep(5)
all_off()

import ms5837 #pressure sensor
import adafruit_icm20x #IMU
import adafruit_ads1x15.ads1115 as ADS #ADC
from adafruit_ads1x15.analog_in import AnalogIn
import board
import time

depth_sensor = ms5837.MS5837_30BA() #Initializes it on the default bus (address = 0x76)
depth_sensor.init()
depth_sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
i2c = board.I2C()
imu = adafruit_icm20x.ICM20649(i2c)
ads = ADS.ADS1115(i2c)

#Depth sensor
def get_depth():
	depth_sensor.read()
	return depth_sensor.depth() #This is in meters

def get_temp():
	depth_sensor.read()
	return depth_sensor.temperature(ms5837.UNITS_Farenheit)

def get_altitude():
	depth_sensor.read()
	return depth_sensor.altitude()

#IMU
def get_acceleration(dir):
	accel = imu.acceleration
	dir = dir.lower()
	if dir == 'x':
		return accel[0]
	elif dir == 'y':
		return accel[1]
	elif dir == 'z':
		return accel[2]

	else:
		return imu.acceleration #This is a tuple in X, Y, and Z

def get_rotation(dir):
	rotation = imu.gyro
	dir = dir.lower()
	if dir == 'x':
		return rotation[0]
	elif dir == 'y':
		return rotation[1]
	elif dir == 'z':
		return rotation[2]
	else:
		return imu.gyro #This is a tuple in X, Y, and Z

#ADC
def read_adc():
	chan = AnalogIn(ads, ADS.P0, ADS.P1)
	ads.gain = 16
	return chan.value


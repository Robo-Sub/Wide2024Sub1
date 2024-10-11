import board
import time
import busio
import adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)
hat.frequency = int(60 * 0.9)  # Check if this frequency is suitable

# Thruster channels configuration
motors = {
    "motor_front_left": hat.channels[14],
    "motor_front_right": hat.channels[15],
    "motor_rear_left": hat.channels[13],
    "motor_rear_right": hat.channels[12],
    "motor_up_left": hat.channels[10],
    "motor_up_right": hat.channels[11]
}

def calc_cycle(throttle):
    if throttle != 0:
        cycle = (throttle + 100) / 200 * 0xffff
    else:
        cycle = 0x7fff
    return int(round(cycle))

def update_thrusters(throttle_values):
    for motor, throttle in throttle_values.items():
        motors[motor].duty_cycle = calc_cycle(throttle)
    time.sleep(0.02)  # control interval

if __name__ == "__main__":
    # Initial off state
    update_thrusters({key: 0 for key in motors.keys()})
    time.sleep(5)

    # Example of simultaneous control
    try:
        start_time = time.time()
        duration = 5  # seconds
        while time.time() - start_time < duration:
            # Define the throttle for each thruster
            throttle_values = {
                "motor_front_left": 2,
                "motor_front_right": 2,
                "motor_rear_left": 2,
                "motor_rear_right": 2,
                "motor_up_left": 2,  # Adjust as necessary
                "motor_up_right": 2
            }
            update_thrusters(throttle_values)
    finally:
        # Ensure all motors are turned off
        update_thrusters({key: 0 for key in motors.keys()})

import time
import math as m
import redis
import struct
import numpy as np
from adafruit_servokit import ServoKit

r = redis.Redis(host='localhost', port=6379, db=0)
kit = ServoKit(channels=16)

#controllable variables
def rget_and_float(name, default = None):
    output = r.get(name)
    if output == None:
        return default
    else:
        return float(output)

rear_diff_locked = int(rget_and_float('rear_diff_locked', 1))
front_diff_locked = int(rget_and_float('front_diff_locked', 1))
gear = int(rget_and_float('gear', 1))
low_battery_voltage = rget_and_float('low_battery_voltage', 3.5)

#----


speed_cap = 45 #percentage of max speed
#steering angle 30 - 150

throttle_stop = 72
throttle_full_forward = 180
throttle_full_reverse = 0

steering_pin = 15
esc_pin = 14

frontdiff_pin = 11
reardiff_pin = 13
gearbox_pin = 12

gear_servo_pos = [0, 60, 110]

rear_diff_servo_pos = [78, 15] #0 locked, 1 open

front_diff_servo_pos = [120, 55] #0 locked, 1 open


def steering_angle(angle):
    if angle > 55:
        angle = 55
    if angle < -55:
        angle = -55
    kit.servo[steering_pin].angle = -angle + 88

def driving_speed_signal(speed):
    if speed > 100:
        speed = 100
    if speed < -72:
        speed = -72
    kit.servo[esc_pin].angle = speed * speed_cap / 100 + 72

driving = True
in_motion_start = time.time()
while driving:
    rear_diff_locked = int(rget_and_float('rear_diff_locked', 1))
    front_diff_locked = int(rget_and_float('front_diff_locked', 1))
    gear = int(rget_and_float('gear', 1))

    kit.servo[gearbox_pin].angle = gear_servo_pos[gear]
    kit.servo[reardiff_pin].angle = rear_diff_servo_pos[rear_diff_locked]
    kit.servo[frontdiff_pin].angle = front_diff_servo_pos[front_diff_locked]

    low_battery_voltage = rget_and_float('low_battery_voltage', 3.5)
    voltages_received = r.get('voltages')
    if voltages_received is None:
        print("no battery info")
        break
    else:
        voltages = np.array(struct.unpack('%sf' %2, voltages_received))
    if voltages.min() < low_battery_voltage:
        print(voltages.min())
        break

    target_speed = r.get('target_speed')
    current_speed_received = r.get('current_speed')
    
    if current_speed_received is not None:
        current_speed = float(current_speed_received)
        #print(current_speed)

    if target_speed is None:
        #print("no driving input received")
        driving_speed_signal(0)
        in_motion_start = time.time()
    else:
        target_speed = float(target_speed)
        if target_speed > 0:
            if current_speed < 0.05 and time.time() - in_motion_start > 2:
                driving_speed_signal(target_speed * 1.5)
                #print("driving faster")
            else:
                driving_speed_signal(target_speed * 1)
                #print("driving normal speed")
        else:
            driving_speed_signal(0)
            #print("stopped")
            in_motion_start = time.time()


    angle_received = r.get('angle')
    if angle_received is None:
        #print("no steering input received")
        steering_angle(0)
    else:
        steering_angle(float(angle_received))
    r.psetex('log_driving_running', 1000, "on")
    time.sleep(0.03) # ???


print("stopping")
driving_speed_signal(0)
steering_angle(-20)
time.sleep(1)
steering_angle(20)
time.sleep(1)
steering_angle(-20)
time.sleep(1)
steering_angle(0)

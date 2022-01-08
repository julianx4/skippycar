import time
import math as m
import redis
import struct
import numpy as np
from adafruit_servokit import ServoKit

r = redis.Redis(host='localhost', port=6379, db=0)
kit = ServoKit(channels=16)

speed_cap = 30 #percentage of max speed
#steering angle 30 - 150

throttle_stop = 72
throttle_full_forward = 180
throttle_full_reverse = 0

steering_pin = 15
esc_pin = 14

voltages=[4.2, 4.2]
voltages_bytes = struct.pack('%sf' %2,* voltages)
r.set('voltages', voltages_bytes)


def steering_angle(angle):
    if angle > 55:
        angle = 55
    if angle < -55:
        angle = -55
    kit.servo[steering_pin].angle = -angle + 88

def driving_speed(speed):
    if speed > 100:
        speed = 100
    if speed < -72:
        speed = -72
    kit.servo[esc_pin].angle = speed * speed_cap / 100 + 72

driving = True
while driving:
    #voltages_received = r.get('voltages')
    #if voltages_received is None:
    #    print("no battery info")
    #    break
    #else:
    #    voltages = np.array(struct.unpack('%sf' %2, voltages_received))
    
    #if voltages.any() < 3.7:
    #    print("battery low")
    #    break

    speed_received = r.get('speed')
    if speed_received is None:
        print("no driving input received")
        driving_speed(0)
    else:
        driving_speed(float(speed_received))

    angle_received = r.get('angle')
    if angle_received is None:
        print("no steering input received")
        steering_angle(0)
    else:
        steering_angle(float(angle_received))


print("stopping")
driving_speed(0)
steering_angle(120)
time.sleep(1)
steering_angle(60)
time.sleep(1)
steering_angle(120)
time.sleep(1)
steering_angle(60)
time.sleep(1)
steering_angle(90)
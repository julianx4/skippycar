import time
import math as m
import redis
import struct
from adafruit_servokit import ServoKit

r = redis.Redis(host='localhost', port=6379, db=0)
kit = ServoKit(channels=16)

speed_cap = 12.5 #percentage of max speed
#steering angle 30 - 150

throttle_stop = 72
throttle_full_forward = 180
throttle_full_reverse = 0

steering_pin = 15
esc_pin = 14

target_coords_bytes = struct.pack('%sf' %3,* [0,0,0])
r.set('target_coords',target_coords_bytes)
voltages=[4.2, 4.2]
voltages_bytes = struct.pack('%sf' %2,* voltages)
r.set('voltages', voltages_bytes)


def steering_angle(angle):
    if angle > 150:
        angle = 150
    if angle < 30:
        angle = 30
    kit.servo[steering_pin].angle = angle

def driving_speed(speed):
    if speed > 100:
        speed = 100
    if speed < -72:
        speed = -72
    kit.servo[esc_pin].angle = speed * speed_cap / 100 + 72

driving = True
while driving:
    if voltages[0] < 3.7 or voltages[1] < 3.7:
        driving = False

    target_coords = struct.unpack('%sf' %3, r.get('target_coords'))
    voltages = struct.unpack('%sf' %2, r.get('voltages'))
    print(voltages)
    print(target_coords)
    time.sleep(0.5)

steering_angle(120)
time.sleep(1)
steering_angle(60)
time.sleep(1)
steering_angle(120)
time.sleep(1)
steering_angle(60)
time.sleep(1)
steering_angle(90)
import time
import math as m
import redis
import struct
import numpy as np
from adafruit_servokit import ServoKit

r = redis.Redis(host='localhost', port=6379, db=0)

car_zero_coords = np.array([0,0,0])
car_coords = np.array([0,0,0])

target_coords = np.array([0,0,2])

yaw_zero = 0
yaw = 0

def angle_and_distance_to_target(car_coords, target_coords, car_yaw_to_world):
    car_vector = np.array([car_coords[0], car_coords[2]])
    target_vector = np.array([target_coords[0], target_coords[2]])
    car2target_vector = target_vector - car_vector
    z_vector=np.array([0,1])
    angle = np.degrees(np.math.atan2(np.linalg.det([car2target_vector,z_vector]),np.dot(car2target_vector,z_vector))) - car_yaw_to_world
    distance = np.linalg.norm(car2target_vector)
    return angle, distance


while True:
    received_car_coords = r.get('car_coords')
    if received_car_coords is None:
        car_zero_coords = car_coords
        print("connection lost")
    else:
        car_coords = np.array(struct.unpack('%sf' %3, received_car_coords))# + car_zero_coords
    
    received_yaw = r.get('yaw')
    if received_yaw is None:
        yaw_zero = yaw
    else:
        yaw = float(received_yaw)# + yaw_zero

    
    angle, distance = angle_and_distance_to_target(car_coords,target_coords,yaw)
    r.psetex('angle', 800, angle)
    print(angle)        
    received_target_coords = r.get('target_coords')
    if received_target_coords is not None:
        target_coords = np.array(struct.unpack('%sf' %3, received_target_coords))# + car_zero_coords   
    if distance > 1:
        r.psetex('speed', 800, 25)

    

    time.sleep(0.1)




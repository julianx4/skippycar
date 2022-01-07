import time
import math as m
import redis
import struct
import numpy as np
import cv2
from adafruit_servokit import ServoKit
from scipy.spatial.transform import Rotation as R

r = redis.Redis(host='localhost', port=6379, db=0)

car_zero_coords = np.array([0,0,0])
car_coords = np.array([0,0,0])

distance = 0

yaw_zero = 0
yaw = 0

target_world_coords = None
target_car_coords = None

car_in_world_coord_x = 0
car_in_world_coord_y = 0
car_in_world_coord_z = 0

rotation_yaw_car_to_world = R.from_rotvec(0 * np.array([0, 1, 0]), degrees=True).as_matrix()
rotation_yaw_world_to_car = R.from_rotvec(-0 * np.array([0, 1, 0]), degrees=True).as_matrix()

def car_coord_to_world_coord(x, y, z):
    c=np.array([x,y,z])
    cx,cy,cz=np.matmul(rotation_yaw_car_to_world,c)
    cx = car_in_world_coord_x + cx
    cy = car_in_world_coord_y + cy
    cz = car_in_world_coord_z + cz
    return cx, cy, cz

def world_coord_to_car_coord(x, y, z):
    c=np.array([x,y,z])
    cx,cy,cz=np.matmul(rotation_yaw_world_to_car,c)
    cx = cx - car_in_world_coord_x
    cy = cy - car_in_world_coord_y
    cz = cz - car_in_world_coord_z
    return cx, cy, cz

def angle_and_distance_to_target(car_in_world, target_world_coords, car_yaw_to_world):
    car_vector = np.array([car_in_world[0], car_in_world[2]])
    target_vector = np.array([target_world_coords[0], target_world_coords[2]])
    car2target_vector = target_vector - car_vector
    z_vector=np.array([0,1])
    angle = np.degrees(np.math.atan2(np.linalg.det([car2target_vector,z_vector]),np.dot(car2target_vector,z_vector))) - car_yaw_to_world
    distance = np.linalg.norm(car2target_vector)
    return angle, distance

should_I_drive = True

while True:
    received_yaw = r.get('yaw')
    if received_yaw is not None:
        yaw = float(received_yaw)

    received_rotation_bytes = r.get('rotation') 
    if received_rotation_bytes is not None:
        rotation = np.array(struct.unpack('%sf' %3, received_rotation_bytes)) 
        pitch = rotation[0]
        roll = rotation[1]
        yaw = rotation[2]
        rotation_yaw_car_to_world = R.from_rotvec(yaw * np.array([0, 1, 0]), degrees=True).as_matrix()
        rotation_yaw_world_to_car = R.from_rotvec(-yaw * np.array([0, 1, 0]), degrees=True).as_matrix()

    received_car_in_world_bytes = r.get('car_in_world') 
    if received_car_in_world_bytes is not None:
        car_in_world = np.array(struct.unpack('%sf' %3, received_car_in_world_bytes))
        car_in_world_coord_x = car_in_world[0]
        car_in_world_coord_y = car_in_world[1]
        car_in_world_coord_z = car_in_world[2]

    received_target_coords = r.get('target_car_coords')
    if received_target_coords is not None:
        target_car_coords = np.array(struct.unpack('%sf' %3, received_target_coords))   
        target_world_coords = car_coord_to_world_coord(target_car_coords[0], target_car_coords[1], target_car_coords[2])

    if should_I_drive is True and target_world_coords is not None:
        angle, distance = angle_and_distance_to_target(car_in_world, target_world_coords, yaw)
        r.psetex('angle', 800, angle)
        #print(angle)       
        if distance > 1:
            r.psetex('speed', 800, 25)

    if target_world_coords is not None:
        print(target_world_coords)
        angle, distance = angle_and_distance_to_target(car_in_world, target_world_coords, yaw)
        print(angle, distance)

    time.sleep(0.2)




import time
import math as m
import redis
import struct
import numpy as np
import cv2
from adafruit_servokit import ServoKit
from scipy.spatial.transform import Rotation as R

r = redis.Redis(host='localhost', port=6379, db=0)

distance = 0

yaw_zero = 0
yaw = 0

target_world_coords = None
target_car_coords = None

rotation_yaw_car_to_world = R.from_rotvec(0 * np.array([0, 1, 0]), degrees=True).as_matrix()
rotation_yaw_world_to_car = R.from_rotvec(-0 * np.array([0, 1, 0]), degrees=True).as_matrix()

mapW=200
mapH=200

def redis_to_map(redis,name):
    encoded = redis.get(name)
    if encoded is None:
        return np.zeros((mapW, mapH, 3), np.uint8)
    else:
        h, w = struct.unpack('>II', encoded[:8])
        array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 1)
        array = cv2.cvtColor(array,cv2.COLOR_GRAY2RGB)
        return array

def obstacle_in_way():
    starttime=time.time()
    map=redis_to_map(r,"map")
    quadrant = []
    #target_coords_x = int(target_car_coords[0] * 100 + mapW/2)
    #target_coords_y = int(mapH - target_car_coords[2] * 100)
        
    for x in range(0,2):
        crop = map[170:190,x*20+80:x*20+100]
        quadrant.append(crop.max()-100)

    for x in range(0,4):
        crop = map[150:170,x*20+60:x*20+80]
        quadrant.append(crop.max()-100)

    for x in range(0,6):
        crop = map[130:150,x*20+40:x*20+60]
        quadrant.append(crop.max()-100)

    for x in range(0,8):
        crop = map[110:130,x*20+20:x*20+40]
        quadrant.append(crop.max()-100)
    max_height = 7
    if quadrant[0] < max_height and quadrant[1] < max_height and quadrant[3] < max_height and quadrant[4] < max_height and quadrant[8] < 7 and quadrant[9] < 7:
        return False
    else:
        return True
    

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
    #car2target_vector = target_vector - car_vector
    car2target_vector = [target_car_coords[0],target_car_coords[2]]
    print(car2target_vector)
    z_vector=np.array([0,1])
    angle = np.degrees(np.math.atan2(np.linalg.det([car2target_vector,z_vector]),np.dot(car2target_vector,z_vector))) #- car_yaw_to_world
    distance = np.linalg.norm(car2target_vector)
    return angle, distance

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
        angle, distance = angle_and_distance_to_target(car_in_world, target_world_coords, yaw)

        if distance > 0.8:
            if not obstacle_in_way():
                print(angle)
                r.psetex('angle', 800, angle*1.2)
                r.psetex('speed', 800, 30)
                print("speed signal sent")
            else:
                print("obstacle, stopping")

    time.sleep(0.01) # ???





import time
import math as m
import redis
import struct
import numpy as np
import cv2
import path_coords as pc
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
        return np.full((mapW,mapH,1),100, np.uint8)
    else:
        h, w = struct.unpack('>II', encoded[:8])
        array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 1)
        array = cv2.cvtColor(array,cv2.COLOR_GRAY2RGB)
        return array


def angle_and_distance_to_target():
    received_target_coords = r.get('target_car_coords')
    if received_target_coords is not None:
        target_car_coords = np.array(struct.unpack('%sf' %3, received_target_coords))   
        car2target_vector = [target_car_coords[0],target_car_coords[2]]
        #print(car2target_vector)
        z_vector=np.array([0,1])
        angle = np.degrees(np.math.atan2(np.linalg.det([car2target_vector,z_vector]),np.dot(car2target_vector,z_vector))) #- car_yaw_to_world
        distance = np.linalg.norm(car2target_vector)       
        print(angle)
        return angle, distance
    else:
        return False, False

def direction_and_speed():
    starttime=time.time()
    
    angle, distance = angle_and_distance_to_target()
    
    if angle is False:
        r.psetex('path', 1000, -1)

        return
    
    map=redis_to_map(r,"map")
    path_costs = [[],[],[],[],[],[],[]]
    
    crop = map[180:200,80:120]
    in_front_of_car = crop.max()-100

    for path in range(0,7):
        target_angle = pc.paths[path]['target_angle']

        height_difference_cost = 0
        square_to_square_cost = 0
        max_height = 0
        
        angle_cost = abs(target_angle - angle) * 2

        for square in range(7, -1, -1):
            x0 = pc.paths[path]['coords'][square][0]
            x1 = pc.paths[path]['coords'][square][1]
            y0 = pc.paths[path]['coords'][square][2]
            y1 = pc.paths[path]['coords'][square][3]
            crop = map[y0:y1,x0:x1]
            min_height = crop.min() - 100
            max_height_previous_step = max_height
            max_height = crop.max() - 100
            
            if max_height - min_height > 10:
                height_difference_cost += 1000
            else:
                height_difference_cost += max_height - min_height

            if abs(max_height - max_height_previous_step) > 10:
                square_to_square_cost += 1000
            else:
                square_to_square_cost += abs(max_height - max_height_previous_step)
            
        total_cost = height_difference_cost + angle_cost + square_to_square_cost
        path_costs[path] = total_cost
            
    current_path = path_costs.index(min(path_costs))
    if min(path_costs) > 1000 or in_front_of_car > 10 or distance < 0.5:
        return
    else:
        r.psetex('path', 1000, current_path)
        r.psetex('speed', 300, 30)
        r.psetex('angle', 1000, pc.paths[current_path]['steering_angle'])

    return current_path

while True:
    direction_and_speed()

    time.sleep(0.05) # ???





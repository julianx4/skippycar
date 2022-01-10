import time
import math as m
import redis
import struct
import numpy as np
import cv2
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
    path_min = [[],[],[],[],[],[],[]]
    path_max = [[],[],[],[],[],[],[]]
    
    crop = map[180:200,80:120]
    in_front_of_car = crop.max()-100

    #path0:
    crop = map[170:190,65:105]
    path_min[0].append(crop.min()-100)
    path_max[0].append(crop.max()-100)

    crop = map[150:170,45:85]
    path_min[0].append(crop.min()-100)
    path_max[0].append(crop.max()-100)
    
    crop = map[140:180,25:45]
    path_min[0].append(crop.min()-100)
    path_max[0].append(crop.max()-100)
    
    crop = map[140:180,5:25]
    path_min[0].append(crop.min()-100)
    path_max[0].append(crop.max()-100)
    
    #path1:
    crop = map[170:190,75:115]
    path_min[1].append(crop.min()-100)
    path_max[1].append(crop.max()-100)
    
    crop = map[150:170,65:105]
    path_min[1].append(crop.min()-100)
    path_max[1].append(crop.max()-100)
    
    crop = map[130:150,55:95]
    path_min[1].append(crop.min()-100)
    path_max[1].append(crop.max()-100)
    
    crop = map[110:130,45:85]
    path_min[1].append(crop.min()-100)
    path_max[1].append(crop.max()-100)
    
    crop = map[90:110,25:65]
    path_min[1].append(crop.min()-100)
    path_max[1].append(crop.max()-100)
    
    crop = map[70:90,5:45]
    path_min[1].append(crop.min()-100)
    path_max[1].append(crop.max()-100)
    
    crop = map[50:70,0:25]
    path_min[1].append(crop.min()-100)
    path_max[1].append(crop.max()-100)
    
    
    #path2:
    crop = map[170:190,75:115]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[150:170,75:115]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[130:150,75:115]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[110:130,65:105]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[90:110,65:105]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[70:90,55:95]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[50:70,45:85]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[30:50,35:75]
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    crop = map[10:3015:65]  
    path_min[2].append(crop.min()-100)
    path_max[2].append(crop.max()-100)
    
    
    #path3:
    crop = map[170:190,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[150:170,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[130:150,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[110:130,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[90:110,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[70:90,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[50:70,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[30:50,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    crop = map[10:30,85:115]
    path_min[3].append(crop.min()-100)
    path_max[3].append(crop.max()-100)
    
    
    #path4:
    crop = map[170:190,85:125]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[150:170,85:125]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[130:150,85:125]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[110:130,95:135]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[90:110,95:135]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[70:90,105:145]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[50:70,115:155]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[30:50,125:165]
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    crop = map[10:30,135:185]  
    path_min[4].append(crop.min()-100)
    path_max[4].append(crop.max()-100)
    
    
    #path5:
    crop = map[170:190,85:125]
    path_min[5].append(crop.min()-100)
    path_max[5].append(crop.max()-100)
    
    crop = map[150:170,95:135]
    path_min[5].append(crop.min()-100)
    path_max[5].append(crop.max()-100)
    
    crop = map[130:150,105:145]
    path_min[5].append(crop.min()-100)
    path_max[5].append(crop.max()-100)
    
    crop = map[110:130,115:155]
    path_min[5].append(crop.min()-100)
    path_max[5].append(crop.max()-100)
    
    crop = map[90:110,135:175]
    path_min[5].append(crop.min()-100)
    path_max[5].append(crop.max()-100)
    
    crop = map[70:90,155:195]
    path_min[5].append(crop.min()-100)
    path_max[5].append(crop.max()-100)
    
    crop = map[50:70,175:200]
    path_min[5].append(crop.min()-100)
    path_max[5].append(crop.max()-100)
    
    
    #path6
    crop = map[170:190,95:145]
    path_min[6].append(crop.min()-100)
    path_max[6].append(crop.max()-100)
    
    crop = map[150:170,115:155]
    path_min[6].append(crop.min()-100)
    path_max[6].append(crop.max()-100)
    
    crop = map[140:180,155:175]
    path_min[6].append(crop.min()-100)
    path_max[6].append(crop.max()-100)
    
    crop = map[140:180,175:195]
    path_min[6].append(crop.min()-100)
    path_max[6].append(crop.max()-100)


    #path costs
    path_angle = [-80, -34, -17, 0, 17, 34, 80]
    path_cost = [0,0,0,0,0,0,0]

    for path in range(0,7):
        #print(path)
        path_cost[path] += abs(path_angle[path] - angle)
        
        for step in range(0,3):
            path_cost[path] += path_max[path][0]
            if path_max[path][0] > 10:
                path_cost[path] += 1000
            else:
                path_cost[path] += path_max[path][0]
            
            if abs(path_max[path][step] - path_min[path][step]) > 10:
                path_cost[path] += 1000
            else:
                path_cost[path] += abs(path_max[path][step] - path_min[path][step])
            
            #print(path_max[path][step])
            if path_max[path][step+1] - path_max[path][step] > 10:
                path_cost[path] += 1000
            else:
                path_cost[path] += abs(path_max[path][step+1] - path_max[path][step])
    #print(path_max)
    current_path = path_cost.index(min(path_cost))
    if min(path_cost) > 1000 or in_front_of_car > 10:
        return
    r.psetex('path', 1000, current_path)
    #print(path_cost)
    return current_path

while True:
    direction_and_speed()

    time.sleep(0.05) # ???





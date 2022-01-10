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


def angle_and_distance_to_target():
    received_target_coords = r.get('target_car_coords')
    if received_target_coords is not None:
        target_car_coords = np.array(struct.unpack('%sf' %3, received_target_coords))   
    car2target_vector = [target_car_coords[0],target_car_coords[2]]
    print(car2target_vector)
    z_vector=np.array([0,1])
    angle = np.degrees(np.math.atan2(np.linalg.det([car2target_vector,z_vector]),np.dot(car2target_vector,z_vector))) #- car_yaw_to_world
    distance = np.linalg.norm(car2target_vector)
    return angle, distance

def direction_and_speed():
    starttime=time.time()
    map=redis_to_map(r,"map")

    path0_min = []
    path0_max = []
    
    path1_min = []
    path1_max = []

    path2_min = []
    path2_max = []

    path3_min = []
    path3_max = []

    path4_min = []
    path4_max = []

    path5_min = []
    path5_max = []

    path6_min = []
    path6_max = []

    #path0:
    crop = map[105:65,170:190]
    path0_min.append(crop.min()-100)
    path0_max.append(crop.max()-100)

    crop = map[85:45,150:170]
    path0_min.append(crop.min()-100)
    path0_max.append(crop.max()-100)
    
    crop = map[45:25,140:180]
    path0_min.append(crop.min()-100)
    path0_max.append(crop.max()-100)
    
    crop = map[25:5,140:180]
    path0_min.append(crop.min()-100)
    path0_max.append(crop.max()-100)
    
    #path1:
    crop = map[115:75,170:190]
    path1_min.append(crop.min()-100)
    path1_max.append(crop.max()-100)
    
    crop = map[105:65,150:170]
    path1_min.append(crop.min()-100)
    path1_max.append(crop.max()-100)
    
    crop = map[95:55,130:150]
    path1_min.append(crop.min()-100)
    path1_max.append(crop.max()-100)
    
    crop = map[85:45,110:130]
    path1_min.append(crop.min()-100)
    path1_max.append(crop.max()-100)
    
    crop = map[65:25,90:110]
    path1_min.append(crop.min()-100)
    path1_max.append(crop.max()-100)
    
    crop = map[45:5,70:90]
    path1_min.append(crop.min()-100)
    path1_max.append(crop.max()-100)
    
    crop = map[25:0,50:70]
    path1_min.append(crop.min()-100)
    path1_max.append(crop.max()-100)
    
    
    #path2:
    crop = map[115:75,170:190]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[115:75,150:170]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[115:75,130:150]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[105:65,110:130]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[105:65,90:110]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[95:55,70:90]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[85:45,50:70]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[75:35,30:50]
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    crop = map[65:15,10:30]  
    path2_min.append(crop.min()-100)
    path2_max.append(crop.max()-100)
    
    
    #path3:
    crop = map[85:115,170:190]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,150:170]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,130:150]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,110:130]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,90:110]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,70:90]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,50:70]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,30:50]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    crop = map[85:115,10:30]
    path3_min.append(crop.min()-100)
    path3_max.append(crop.max()-100)
    
    
    #path4:
    crop = map[85:125,170:190]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[85:125,150:170]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[85:125,130:150]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[95:135,110:130]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[95:135,90:110]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[105:145,70:90]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[115:155,50:70]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[125:165,30:50]
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    crop = map[135:185,10:30]  
    path4_min.append(crop.min()-100)
    path4_max.append(crop.max()-100)
    
    
    #path5:
    crop = map[85:125,170:190]
    path5_min.append(crop.min()-100)
    path5_max.append(crop.max()-100)
    
    crop = map[95:135,150:170]
    path5_min.append(crop.min()-100)
    path5_max.append(crop.max()-100)
    
    crop = map[105:145,130:150]
    path5_min.append(crop.min()-100)
    path5_max.append(crop.max()-100)
    
    crop = map[115:155,110:130]
    path5_min.append(crop.min()-100)
    path5_max.append(crop.max()-100)
    
    crop = map[135:175,90:110]
    path5_min.append(crop.min()-100)
    path5_max.append(crop.max()-100)
    
    crop = map[155:195,70:90]
    path5_min.append(crop.min()-100)
    path5_max.append(crop.max()-100)
    
    crop = map[175:200,50:70]
    path5_min.append(crop.min()-100)
    path5_max.append(crop.max()-100)
    
    
    #path6
    crop = map[95:145,170:190]
    path6_min.append(crop.min()-100)
    path6_max.append(crop.max()-100)
    
    crop = map[115:155,150:170]
    path6_min.append(crop.min()-100)
    path6_max.append(crop.max()-100)
    
    crop = map[155:175,140:180]
    path6_min.append(crop.min()-100)
    path6_max.append(crop.max()-100)
    
    crop = map[175:195,140:180]
    path6_min.append(crop.min()-100)
    path6_max.append(crop.max()-100)

    angle, distance = angle_and_distance_to_target()

    #path costs
    path_angle=[-80, -34, -17, 0, 17, 34, 80]

    path_cost = [0,0,0,0,0,0,0]

    for path in range(len(path_cost)):
        path_cost[path] += path_angle[path] - angle

    for step in range(0,3):
        path_cost[0] += path0_min


while True:


    time.sleep(0.01) # ???





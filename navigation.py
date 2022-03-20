import time
import math as m
import redis
import struct
import numpy as np
import cv2
import curved_paths_coords as pc
from scipy.spatial.transform import Rotation as R

r = redis.Redis(host='localhost', port=6379, db=0)

#controllable variables

def rget_and_float(name, default = None):
    output = r.get(name)
    if output == None:
        return default
    else:
        return float(output)

angle_deviation_cost_factor = rget_and_float('angle_deviation_cost_factor', 2)
driving_speed = rget_and_float('driving_speed', 30)
max_climb_height = rget_and_float('max_climb_height', 10)
min_speed = rget_and_float('min_speed', 0.05)
min_speed_increase_factor = rget_and_float('min_speed_increase_factor', 1.5)
square_range = int(rget_and_float('square_range', 6))
obstacle_stop_height = rget_and_float('obstacle_stop_height', 15)
target_stop_distance = rget_and_float('target_stop_distance', 0.9)
square_to_square_cost_factor = rget_and_float('square_to_square_cost_factor', 10)
#----

distance = 0

yaw_zero = 0
yaw = 0

target_world_coords = None
target_car_coords = None

in_front_of_car = 0

rotation_yaw_car_to_world = R.from_rotvec(0 * np.array([0, 1, 0]), degrees=True).as_matrix()
rotation_yaw_world_to_car = R.from_rotvec(-0 * np.array([0, 1, 0]), degrees=True).as_matrix()

mapW=400
mapH=400

target_speed = None

def redis_to_map(redis,name):
    encoded = redis.get(name)
    if encoded is None:
        return np.full((mapW,mapH,1),100, np.uint8)
    else:
        h, w = struct.unpack('>II', encoded[:8])
        received_array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 1)
        #received_array = cv2.cvtColor(received_array,cv2.COLOR_GRAY2RGB)
        
        return received_array

def angle_and_distance_to_target():
    received_target_coords = r.get('target_world_coords')
    received_car_in_world_coords = r.get('car_in_world')
    rotation_received = r.get('rotation')
    
    if received_target_coords is not None and received_car_in_world_coords is not None and rotation_received is not None:
        bla, bla, car_yaw_to_world = np.array(struct.unpack('%sf' %3, rotation_received))
        target_world_coords = np.array(struct.unpack('%sf' %3, received_target_coords))
        car_world_coords = np.array(struct.unpack('%sf' %3, received_car_in_world_coords))
        car2target_vector3D = target_world_coords - car_world_coords
        car2target_vector = [car2target_vector3D[0],car2target_vector3D[2]]
        
        z_vector=np.array([0,1])
        angle = np.degrees(np.math.atan2(np.linalg.det([car2target_vector,z_vector]),np.dot(car2target_vector,z_vector))) - car_yaw_to_world
        distance = np.linalg.norm(car2target_vector)     
        r.psetex('log_target_distance', 1000, distance)
        r.psetex('log_target_angle', 1000, angle)
        #print("angle", angle)  
        return angle, distance
    else:
        return False, False

while True:
    starttime = time.time()
    angle_deviation_cost_factor = rget_and_float('angle_deviation_cost_factor', 2)
    driving_speed = rget_and_float('driving_speed', 30)
    max_climb_height = rget_and_float('max_climb_height', 10)
    min_speed = rget_and_float('min_speed', 0.05)
    min_speed_increase_factor = rget_and_float('min_speed_increase_factor', 1.5)
    square_range = int(rget_and_float('square_range', 6))
    obstacle_stop_height = rget_and_float('obstacle_stop_height', 15)
    target_stop_distance = rget_and_float('target_stop_distance', 0.9)
    square_to_square_cost_factor = rget_and_float('square_to_square_cost_factor', 10)
    map=redis_to_map(r,"map")
    path_costs = [[],[],[],[],[],[],[],[],[],[],[]]
    
    crop = map[230:241,188:212]
    #crop = map[230:250,191:209]
    in_front_of_car = crop.max() - 100
    r.psetex('log_in_front_of_car', 1000, float(in_front_of_car))
    angle, distance = angle_and_distance_to_target()
    if angle is False:
        r.psetex('path', 1000, -1)


    #0 straight
    #1 - 5 right
    #6 - 10 left
    received_target_coords = r.get('target_world_coords')
    if received_target_coords is not None:
        target_world_coords = np.array(struct.unpack('%sf' %3, received_target_coords))


        for path in range(0, 11):
            if path > 5:
                path_lookup = path - 5
                l = -1
                target_angle = -pc.paths[path_lookup]['target_angle']
                
            else:
                l = 1
                path_lookup = path
                target_angle = pc.paths[path_lookup]['target_angle']
            
            height_difference_cost = 0
            square_to_square_cost = 0
            max_height = 0
            
            angle_cost = abs(target_angle - angle) * angle_deviation_cost_factor
            
            for square in range(0, square_range):
                #print(path,square)
                
                x0 = int(l * pc.paths[path_lookup]['coords'][square][0] / 10 + mapW / 2)
                y0 = mapH - int(pc.paths[path_lookup]['coords'][square][1] / 10 + 150)
                x1 = int(l * pc.paths[path_lookup]['coords'][square][2] / 10 + mapW / 2)
                y1 = mapH - int(pc.paths[path_lookup]['coords'][square][3] / 10 + 150)
                
                x2 = int(l * pc.paths[path_lookup]['coords'][square + 1][0] / 10 + mapW / 2)
                y2 = mapH - int(pc.paths[path_lookup]['coords'][square + 1][1] / 10 + 150)
                x3 = int(l * pc.paths[path_lookup]['coords'][square + 1][2] / 10 + mapW / 2)
                y3 = mapH - int(pc.paths[path_lookup]['coords'][square + 1][3] / 10 + 150)
                poly = np.array([[x0,y0],[x1,y1],[x3,y3],[x2,y2]])
                
                poly = poly.reshape((-1, 1, 2))
                
                
                mask = np.zeros(map.shape, dtype=np.uint8)
                cv2.fillPoly(mask,[poly],(255,255,255))
                crop = cv2.bitwise_and(map,mask)
                bg = np.ones_like(crop, np.uint8)*100
                bgmask= cv2.bitwise_not(mask)
                bg = cv2.bitwise_and(bg,bgmask)
                crop = crop + bg


                min_height = crop.min() - 100
                max_height_previous_step = max_height
                max_height = crop.max() - 100
                if square > 4:
                    height_difference_cost += (max_height - min_height) / 4
                    square_to_square_cost += (abs(max_height - max_height_previous_step)) / 4
                
                else:
                    #if max_height - min_height > 10:
                    #    height_difference_cost += 1000
                    #else:
                    #    height_difference_cost += max_height - min_height

                    if abs(max_height - max_height_previous_step) > max_climb_height:
                        square_to_square_cost += 1000
                    else:
                        square_to_square_cost += abs(max_height - max_height_previous_step) * square_to_square_cost_factor
            
            total_cost = height_difference_cost + angle_cost + square_to_square_cost
            path_costs[path] = total_cost

        minpathcost = min(path_costs)
        r.psetex('path_min_cost', 1000, str(minpathcost))
        current_path = path_costs.index(minpathcost)
        #print("current path", current_path, minpathcost)

        if min(path_costs) > 1000 or in_front_of_car > obstacle_stop_height or distance < target_stop_distance:
            #print("stopping: obstacle", in_front_of_car, "distance to target",distance, "min path cost",minpathcost)
            r.set('target_speed', 0)
            target_speed = None

        else:
            r.psetex('path', 1000, current_path)

            current_speed_received = r.get('current_speed')

            if current_speed_received is not None:
                current_speed = float(current_speed_received)

            if target_speed is None:
                #print("no driving input received")
                in_motion_start = time.time()

            if current_speed < min_speed and time.time() - in_motion_start > 2:
                #print("driving faster")
                r.psetex('target_speed', 1000, driving_speed * min_speed_increase_factor)
            else:
                #print("driving normal speed")
                r.psetex('target_speed', 1000, driving_speed)
            if current_path > 5:
                path_lookup = current_path - 5
                steering_angle = -pc.paths[path_lookup]['steering_angle']
            else:
                path_lookup = current_path
                steering_angle = pc.paths[path_lookup]['steering_angle']

            r.psetex('angle', 1000, steering_angle)

    r.psetex('log_navigation_running', 1000, "on")
    time.sleep(0.1)

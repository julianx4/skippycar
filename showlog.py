from xmlrpc.client import boolean
import redis
import os
import time
import numpy as np
import struct

from tabulate import tabulate

r = redis.Redis(host='localhost', port=6379, db=0)
while True:
    log_sensing_time_received = r.get('log_sensing_time')
    if log_sensing_time_received is not None:
        log_sensing_time = round(float(log_sensing_time_received),2)
    else:
        log_sensing_time = 0    

    log_target_distance_received = r.get('log_target_distance')
    if log_target_distance_received is not None:
        log_target_distance = round(float(log_target_distance_received),2)
    else:
        log_target_distance = "None"

    log_target_angle_received = r.get('log_target_angle')
    if log_target_angle_received is not None:
        log_target_angle = round(float(log_target_angle_received),2)
    else:
        log_target_angle = "None"
    
    log_path_received = r.get('path')
    if log_path_received is not None:
        log_path = float(log_path_received)
    else:
        log_path = "None"

    log_path_min_cost_received = r.get('path_min_cost')
    if log_path_min_cost_received is not None:
        log_path_min_cost = round(float(log_path_min_cost_received),2)
    else:
        log_path_min_cost = "None"

    log_current_speed_received = r.get('current_speed')
    if log_current_speed_received is not None:
        log_current_speed = round(float(log_current_speed_received),2)
    else:
        log_current_speed = "None"
    
    log_in_front_of_car_received = r.get('log_in_front_of_car')
    if log_in_front_of_car_received is not None:
        log_in_front_of_car = float(log_in_front_of_car_received)
    else:
        log_in_front_of_car = "None"

    
    voltages_received = r.get('voltages')
    if voltages_received is not None:
        voltages = np.round(np.array(struct.unpack('%sf' %2, voltages_received)),2)
    else:
        voltages = [0,0]

    os.system('clear')
    print(tabulate([\
        ['battery voltages', voltages[0], voltages[1]], \
        ['sensing time', log_sensing_time],\
        ['target distance, angle', log_target_distance, log_target_angle],\
        ['current path', log_path],\
        ['path min cost', log_path_min_cost],\
        ['current speed', log_current_speed],\
        ['obstacle height', log_in_front_of_car]\
        ]))
    time.sleep(0.2)
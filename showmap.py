import numpy as np
import redis
import struct
import cv2
import time
import curved_paths_coords as pc

r = redis.Redis(host='localhost', port=6379, db=0)

mapW = 400
mapH = 400

last_time=0

font = cv2.FONT_HERSHEY_SIMPLEX

map_refresh = 0.1 # interval between map refresh

map = np.full((mapW,mapH,3),100, np.uint8)

def redis_to_map(redis,name):
    encoded = redis.get(name)
    if encoded is None:
        return np.full((mapW,mapH,3),100, np.uint8)
    else:
        h, w = struct.unpack('>II', encoded[:8])
        array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 1)
        array = cv2.cvtColor(array,cv2.COLOR_GRAY2RGB)
        return array

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
    
    log_target_distance_angle = str(log_target_distance) + " " + str(log_target_angle)
    
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

    voltages1_and_2 = str(voltages[0]) + " " + str(voltages[1])

    log_uptime_received = r.get('log_uptime')
    if log_uptime_received is not None:
        log_uptime = int(float(log_uptime_received))
    else:
        log_uptime = "None"

    log_sensing_running_received = r.get('log_sensing_running')
    if log_sensing_running_received is not None:
        log_sensing_running = str(log_sensing_running_received.decode("utf-8") )
    else:
        log_sensing_running = "off"

    log_navigation_running_received = r.get('log_navigation_running')
    if log_navigation_running_received is not None:
        log_navigation_running = str(log_navigation_running_received.decode("utf-8") )
    else:
        log_navigation_running = "off"

    log_batterymeter_running_received = r.get('log_batterymeter_running')
    if log_batterymeter_running_received is not None:
        log_batterymeter_running = str(log_batterymeter_running_received.decode("utf-8") )
    else:
        log_batterymeter_running = "off"

    log_driving_running_received = r.get('log_driving_running')
    if log_driving_running_received is not None:
        log_driving_running = str(log_driving_running_received.decode("utf-8") )
    else:
        log_driving_running = "off"


        
    last_time = time.time()
    map = redis_to_map(r, "map")

    cv2.rectangle(map,(187,242),(213,305),(0, 100, 255),-1) #draw car
    visible_cone = np.array([[213, 242], [187, 242], [0, 0], [400, 0]], np.int32)
    visible_cone = visible_cone.reshape((-1, 1, 2))
    cv2.polylines(map, [visible_cone], True, (255,255,255), 1)

    color_path = (0,255,0)
    path_received = r.get('path')
    if path_received is None:
        pass    
    elif int(path_received) == -1:
        pass
    else:
        path = int(path_received)
        if path > 5:
            path_lookup = path - 5
            l = -1
        else:
            path_lookup = path
            l = 1
        for square in range(0, 4):
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
            cv2.polylines(map,[poly],True,(255,255,255),1)

    received_target_coords = r.get('target_car_coords')
    if received_target_coords is not None:
        target_car_coords = np.array(struct.unpack('%sf' %3, received_target_coords))   
        mx = int(target_car_coords[0] * 100 + mapW / 2)
        my = int(mapH - target_car_coords[2] * 100)	
        cv2.line(map, (int(mapW/2), mapH - 150), (mx, my - 150), (0,0,255), thickness=3)


    topic_left=['sensing', \
        'navigation',\
        'batterymeter',\
        'driving'\
    ]
    logs_left=[log_sensing_running, \
        log_navigation_running,\
        log_batterymeter_running,\
        log_driving_running\
    ]

    topic_right=['battery voltages', \
        'sensing time',\
        'target dist, angle',\
        'current path',\
        'path min cost',\
        'current speed',\
        'obstacle height',\
        'uptime'\
    ]
    logs_right=[voltages1_and_2, \
        log_sensing_time,\
        log_target_distance_angle,\
        log_path,\
        log_path_min_cost,\
        log_current_speed,\
        log_in_front_of_car,\
        log_uptime\
    ]
    count = 1
    for text in topic_left:
        count +=1
        cv2.putText(map, str(text), (20, 300 + 10 * count), font, 0.4, (255,255,255), 1)

    count = 1
    for text in logs_left:
        count +=1
        cv2.putText(map, str(text), (140, 300 + 10 * count), font, 0.4, (255,255,255), 1)

    count = 1
    for text in topic_right:
        count +=1
        cv2.putText(map, str(text), (187, 300 + 10 * count), font, 0.4, (255,255,255), 1)

    count = 1
    for text in logs_right:
        count +=1
        cv2.putText(map, str(text), (310, 300 + 10 * count), font, 0.4, (255,255,255), 1)


    cv2.namedWindow('map', cv2.WINDOW_NORMAL)
    cv2.imshow('map', map)
    time.sleep(map_refresh)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

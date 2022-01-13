import numpy as np
import redis
import struct
import cv2
import time
import path_coords as pc

r = redis.Redis(host='localhost', port=6379, db=0)

mapW = 400
mapH = 400

last_time=0

font = cv2.FONT_HERSHEY_SIMPLEX

map_refresh = 0.25 # interval between map refresh

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

    last_time = time.time()
    map = redis_to_map(r, "map")

    cv2.rectangle(map,(187,242),(213,305),(255),-1) #draw car
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
        for square in range(7, -1, -1):
            x0 = pc.paths[path]['coords'][square][0] + 100
            x1 = pc.paths[path]['coords'][square][1] + 100
            y0 = pc.paths[path]['coords'][square][2] + 50
            y1 = pc.paths[path]['coords'][square][3] + 50
            cv2.rectangle(map,(x0,y0),(x1,y1),color_path, 1)


    received_target_coords = r.get('target_car_coords')
    if received_target_coords is not None:
        target_car_coords = np.array(struct.unpack('%sf' %3, received_target_coords))   
        mx = int(target_car_coords[0] * 100 + mapW / 2)
        my = int(mapH - target_car_coords[2] * 100)	
        cv2.circle(map, (mx,my), 1, (0,0,255), thickness=-1, lineType=8, shift=0)
        cv2.line(map, (int(mapW/2), mapH), (mx, my), (0,0,255), thickness=3)


    cv2.namedWindow('map', cv2.WINDOW_NORMAL)
    cv2.imshow('map', map)
    time.sleep(map_refresh)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

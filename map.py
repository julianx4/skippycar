import numpy as np
import redis
import struct
import cv2
import time

r = redis.Redis(host='localhost', port=6379, db=0)

mapW=200
mapH=200

last_time=0

font = cv2.FONT_HERSHEY_SIMPLEX

map_refresh = 1 # interval between map refresh

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

    cv2.rectangle(map,(87,192),(113,200),(255),-1) #draw car

    for x in range(0,2):
        cv2.rectangle(map,(x*20+80,170),(x*20+100,190),(0,255,0),thickness=1)
        cv2.putText(map,str(x),(x*20+80+5,185), font, 0.3,(255,255,255),1)

    for x in range(0,4):
        cv2.rectangle(map,(x*20+60,150),(x*20+80,170),(0,255,0),thickness=1)
        cv2.putText(map,str(x+2),(x*20+60+5,165), font, 0.3,(255,255,255),1)

    for x in range(0,6):
        cv2.rectangle(map,(x*20+40,130),(x*20+60,150),(0,255,0),thickness=1)
        cv2.putText(map,str(x+6),(x*20+40+5,145), font, 0.3,(255,255,255),1)

    for x in range(0,8):
        cv2.rectangle(map,(x*20+20,110),(x*20+40,130),(0,255,0),thickness=1)
        cv2.putText(map,str(x+12),(x*20+20+5,125), font, 0.3,(255,255,255),1)
    
    cv2.rectangle(map,(60,0),(140,110),(0,255,0),thickness=1)
    cv2.putText(map,str(20),(95,95), font, 0.3,(255,255,255),1)

    cv2.line(map, (87, 0), (87, 200), (255, 0, 0), thickness=1)
    cv2.line(map, (113, 0), (113, 200), (255, 0, 0), thickness=1)

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

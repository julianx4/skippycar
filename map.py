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

    color_path = (0,255,0)
    path_received = r.get('path')
    if path_received is None:
        path = -1
    else:
        path = float(path_received)

    if path == 0:
        cv2.rectangle(map,(65,170),(105,190),color_path, 1)
        cv2.rectangle(map,(45,150),(85,170),color_path, 1)
        cv2.rectangle(map,(25,140),(45,180),color_path, 1)
        cv2.rectangle(map,(5,140),(25,189),color_path, 1)


    if path == 1:
        cv2.rectangle(map,(75,170),(115,190),color_path, 1)
        cv2.rectangle(map,(65,150),(105,170),color_path, 1)
        cv2.rectangle(map,(55,130),(95,150),color_path, 1)
        cv2.rectangle(map,(45,110),(85,130),color_path, 1)
        cv2.rectangle(map,(25,90),(65,110),color_path, 1)
        cv2.rectangle(map,(5,70),(45,90),color_path, 1)
        cv2.rectangle(map,(0,50),(25,70),color_path, 1)

    if path == 2:
        cv2.rectangle(map,(75,170),(115,190),color_path, 1)
        cv2.rectangle(map,(75,150),(115,170),color_path, 1)
        cv2.rectangle(map,(75,130),(115,150),color_path, 1)
        cv2.rectangle(map,(65,110),(105,130),color_path, 1)
        cv2.rectangle(map,(65,90),(105,110),color_path, 1)
        cv2.rectangle(map,(55,70),(95,90),color_path, 1)
        cv2.rectangle(map,(45,50),(85,70),color_path, 1)
        cv2.rectangle(map,(35,30),(75,50),color_path, 1)
        cv2.rectangle(map,(15,10),(65,30),color_path, 1)

    if path == 3:
        cv2.rectangle(map,(85,170),(115,190),color_path, 1)
        cv2.rectangle(map,(85,150),(115,170),color_path, 1)
        cv2.rectangle(map,(85,130),(115,150),color_path, 1)
        cv2.rectangle(map,(85,110),(115,130),color_path, 1)
        cv2.rectangle(map,(85,90),(115,110),color_path, 1)
        cv2.rectangle(map,(85,70),(115,90),color_path, 1)
        cv2.rectangle(map,(85,50),(115,70),color_path, 1)
        cv2.rectangle(map,(85,30),(115,50),color_path, 1)
        cv2.rectangle(map,(85,10),(115,30),color_path, 1)

    if path == 4:
        cv2.rectangle(map,(85,170),(125,190),color_path, 1)
        cv2.rectangle(map,(85,150),(125,170),color_path, 1)
        cv2.rectangle(map,(85,130),(125,150),color_path, 1)
        cv2.rectangle(map,(95,110),(135,130),color_path, 1)
        cv2.rectangle(map,(95,90),(135,110),color_path, 1)
        cv2.rectangle(map,(105,70),(145,90),color_path, 1)
        cv2.rectangle(map,(115,50),(155,70),color_path, 1)
        cv2.rectangle(map,(125,30),(165,50),color_path, 1)
        cv2.rectangle(map,(135,10),(185,30),color_path, 1)

    if path == 5:   
        cv2.rectangle(map,(85,170),(125,190),color_path, 1)
        cv2.rectangle(map,(95,150),(135,170),color_path, 1)
        cv2.rectangle(map,(105,130),(145,150),color_path, 1)
        cv2.rectangle(map,(115,110),(155,130),color_path, 1)
        cv2.rectangle(map,(135,90),(175,110),color_path, 1)
        cv2.rectangle(map,(155,70),(195,90),color_path, 1)
        cv2.rectangle(map,(175,50),(200,70),color_path, 1)

    if path == 6:
        cv2.rectangle(map,(95,170),(135,190),color_path, 1)
        cv2.rectangle(map,(115,150),(155,170),color_path, 1)
        cv2.rectangle(map,(155,140),(175,180),color_path, 1)
        cv2.rectangle(map,(175,140),(195,189),color_path, 1)


    #for x in range(0,2):
    #    cv2.rectangle(map,(x*20+80,170),(x*20+100,190),(0,255,0),thickness=1)
    #    cv2.putText(map,str(x),(x*20+80+5,185), font, 0.3,(255,255,255),1)

    #for x in range(0,4):
    #    cv2.rectangle(map,(x*20+60,150),(x*20+80,170),(0,255,0),thickness=1)
    #    cv2.putText(map,str(x+2),(x*20+60+5,165), font, 0.3,(255,255,255),1)

    #for x in range(0,6):
    #    cv2.rectangle(map,(x*20+40,130),(x*20+60,150),(0,255,0),thickness=1)
    #    cv2.putText(map,str(x+6),(x*20+40+5,145), font, 0.3,(255,255,255),1)

    #for x in range(0,8):
    #    cv2.rectangle(map,(x*20+20,110),(x*20+40,130),(0,255,0),thickness=1)
    #    cv2.putText(map,str(x+12),(x*20+20+5,125), font, 0.3,(255,255,255),1)
    
    #cv2.rectangle(map,(60,0),(140,110),(0,255,0),thickness=1)
    #cv2.putText(map,str(20),(95,95), font, 0.3,(255,255,255),1)

    #cv2.line(map, (87, 0), (87, 200), (255, 0, 0), thickness=1)
    #cv2.line(map, (113, 0), (113, 200), (255, 0, 0), thickness=1)

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

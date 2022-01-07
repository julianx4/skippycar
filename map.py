import numpy as np
import redis
import struct
import cv2
import time

r = redis.Redis(host='localhost', port=6379, db=0)

mapW=200
mapH=200

last_time=0

map_refresh = 0.5 # interval between map refresh

map = np.zeros((mapW,mapH,3), np.uint8)

def redis_to_map(redis,name):
    encoded = redis.get(name)
    if encoded is None:
        return np.zeros((mapW, mapH, 3), np.uint8)
    else:
        h, w = struct.unpack('>II', encoded[:8])
        array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 3)
        return array

while True:
    if time.time() - last_time > map_refresh:
        last_time = time.time()
        map = redis_to_map(r, "map")

    cv2.namedWindow('map', cv2.WINDOW_NORMAL)
    cv2.imshow('map', map)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

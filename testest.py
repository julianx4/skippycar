
import redis
import struct
import time

r = redis.Redis(host='localhost', port=6379, db=0)

while True:
    print(struct.unpack('%sf' %3, r.get('target_world_coords')))
    print("bla")
    time.sleep(0.5)
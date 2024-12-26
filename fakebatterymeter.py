import time
import redis
import struct

r = redis.Redis(host='localhost', port=6379, db=0)
while True:

    cell1 = 4
    cell2 = 4
    
    VoltageLastMeasured=time.time()
    voltages_bytes = struct.pack('%sf' %2,* [cell1, cell2])
    print(struct.unpack('%sf' %2,voltages_bytes))
    r.setex('voltages', 15, voltages_bytes)
    r.setex('voltage_cell1', 15, cell1)
    r.setex('voltage_cell2', 15, cell2)
    r.psetex('log_batterymeter_running', 4000, "on")

    time.sleep(10)

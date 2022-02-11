import time
import Adafruit_ADS1x15
import redis
import struct
# yellow SDA
# orange SCL
r = redis.Redis(host='localhost', port=6379, db=0)
adc = Adafruit_ADS1x15.ADS1115(address=0x48, busnum=1)
GAIN = 2/3
VoltageLastMeasured=0
VoltageMeasuringPeriod=3
while True:

    read1=adc.read_adc(1, gain=GAIN) * 2
    read3=adc.read_adc(3, gain=GAIN)
    cell1 = (read1-read3)/32768*6.144
    cell2 = (read3)/32768*6.144
    
    VoltageLastMeasured=time.time()
    voltages_bytes = struct.pack('%sf' %2,* [cell1, cell2])
    print(struct.unpack('%sf' %2,voltages_bytes))
    r.setex('voltages', 15, voltages_bytes)

    time.sleep(VoltageMeasuringPeriod)

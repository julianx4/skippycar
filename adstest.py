import time
import Adafruit_ADS1x15

# yellow SDA
# orange SCL

adc = Adafruit_ADS1x15.ADS1115(address=0x48, busnum=1)
GAIN = 2/3
VoltageLastMeasured=0
VoltageMeasuringPeriod=1
while True:
    if time.time()-VoltageLastMeasured>VoltageMeasuringPeriod:
        read1=adc.read_adc(1, gain=GAIN) * 2
        read3=adc.read_adc(3, gain=GAIN)
        cell1 = (read1-read3)/32768*6.144
        cell2 = (read3)/32768*6.144
        VoltageLastMeasured=time.time()
        print("Cell 1: ", round(cell1,1),"V")
        print("Cell 2: ", round(cell2,1),"V")



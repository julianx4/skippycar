# skippycar
Selfdriving RC car with realsense D435 and T265 camera that follows an april tag

Functions of the car:
sensing.py
- uses the T265 camera to understand where the car is positioned in the world
- uses the D435 camera to see in front of it and create a height map
- uses the D435 camera to detect an april tag
- output: elevation map, world and target coordinates

navigation.py
- takes the information from sensing.py to decide the best path ahead towards the april tag. It choses the path based on the height map ahead. (Certain elevation changes are ok, as the car is able to drive over obstacles of a certain height)
- output: steering angle, speed and for display purposes, desired path based on target position and elevation map ahead

batterymeter.py
- uses and ADS1116 to read the voltage of the RC car battery

driving.py
- uses and ADAFRUIT 16 channel servo HAT to control servo and ESC
- takes the input of navigation.py and sends the signal to servo and ESC
- also takes the input of batterymeter.py. If the voltage falls to a certain level, the car will stop

showmap.py
- uses CV2 to display elevation map, driving path and target direction and a lot of debugging information.

Communication between all scripts is done via redis.

Unfortunately the T265 camera doesn't work properly on the Raspberry Pi4. Intel refuses to continue to support the T265 (https://github.com/IntelRealSense/librealsense/issues/6362#issuecomment-697971303). It was adressed on the raspberrypi forum, but also there unfortunately it was a dead end (https://forums.raspberrypi.com/viewtopic.php?t=281665)
This means that the script running the cameras crashes regularly any time between 20 seconds and 10 minutes. Running it as a service softens the issue, but it still sucks.
The raspberry pi 3 isn't really powerful enough. On the pi 4 the main script takes around 0.06 to 0.12 seconds each pass on the pi3 it takes 3, sometimes 5 times as long. This isn't enough. Probably some performance enhancement is possible though.

Ideas for the future:
- follow a person instead of april tag by using object detection / classification
- add a GPS to drive a pre-planned path
- add the option to use an XBox controller to control
- somehow fork OpenHD and transfer a live image
- ...
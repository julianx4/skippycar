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

driving.py
- uses and ADAFRUIT 16 channel servo HAT to control servo and ESC
- takes the input of navigation.py and sends the signal to servo and ESC
- also takes the input of batterymeter.py. If the voltage falls to a certain level, the car will stop

showmap.py
- uses CV2 to display elevation map, driving path and target direction

batterymeter.py
- uses and ADS1116 to read the voltage of the RC car battery
- output battery voltage


Communication between all scripts is done via redis.

Unfortunately the T265 camera doesn't work on the Raspberry Pi4 and the Raspberry Pi3 seems not powerful enough to manage all this, so it isn't working.

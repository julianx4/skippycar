from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

speed_cap = 12.5 #percentage of max speed
#steering angle 30 - 150

throttle_stop = 72
throttle_full_forward = 180
throttle_full_reverse = 0

steering_pin = 15
esc_pin = 14

def steering_angle(angle):
    if angle > 150:
        angle = 150
    if angle < 30:
        angle = 30
    kit.servo[steering_pin].angle = angle

def driving_speed(speed):
    if speed > 100:
        speed = 100
    if speed < -72:
        speed = -72
    kit.servo[esc_pin].angle = speed * speed_cap / 100 + 72

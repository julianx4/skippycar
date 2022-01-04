from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(1000, 2000)

speed_cap = 50 #percentage of max speed

throttle_stop = 72
throttle_full_forward = 180
throttle_full_reverse = 0

steering_pin = 15
esc_pin = 0

def steering_angle(angle):
    kit.servo[steering_pin].angle = angle

def driving_speed(speed):
    if speed > 100:
        speed = 100
    if speed < -72:
        speed = -72
    kit.servo[steering_pin].angle = speed * speed_cap / 100 + 72

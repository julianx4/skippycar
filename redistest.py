import redis

r = redis.Redis(host='localhost', port=6379, db=0)

r.set('angle_deviation_cost_factor', 2)
r.set('square_to_square_cost', 10)
r.set('square_range', 6)
r.set('driving_speed' , 30)
r.set('max_climb_height', 10)
r.set('low_battery_voltage' , 3.5)
r.set('min_speed' ,0.05)
r.set('min_speed_increase_factor' , 1.5)
r.set('obstacle_stop_height' , 15)
r.set('target_stop_distance' , 0.9)
r.set('target_memory_time' , 1)
r.set('map_clear_all_interval' , 3)
r.set('map_clear_visible_cone_interval' , 0.5)
r.set('depth_pixel_horizontal_raster' , 50)
r.set('depth_pixel_vertical_raster' , 10)
r.set('ignore_above_height' , 0.4)
r.set('rear_diff_locked' , 1)
r.set('front_diff_locked' , 1)
r.set('gear' , 1)
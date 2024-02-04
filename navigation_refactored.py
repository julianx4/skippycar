import time
import math as m
import struct
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import redis
import curved_paths_coords as pc

class PathNavigator:
    def __init__(self):
        self.redis = redis.Redis(host='localhost', port=6379, db=0)
        self.load_configuration()
        self.map_width = 400
        self.map_height = 400
        self.target_speed = None
        self.in_front_of_car = 0

    def load_configuration(self):
        self.angle_deviation_cost_factor = self.rget_and_float('angle_deviation_cost_factor', 2)
        self.angle_deviation_expo = self.rget_and_float('angle_deviation_expo', 1)
        self.driving_speed = self.rget_and_float('driving_speed', 30)
        self.max_climb_height = self.rget_and_float('max_climb_height', 10)
        self.min_speed = self.rget_and_float('min_speed', 0.05)
        self.min_speed_increase_factor = self.rget_and_float('min_speed_increase_factor', 1.5)
        self.square_range = int(self.rget_and_float('square_range', 6))
        self.obstacle_stop_height = self.rget_and_float('obstacle_stop_height', 15)
        self.target_stop_distance = self.rget_and_float('target_stop_distance', 0.9)
        self.square_to_square_cost_factor = self.rget_and_float('square_to_square_cost_factor', 10)
        self.map_base_height = int(self.rget_and_float('map_base_height', 100))


    def rget_and_float(self, name, default=None):
        output = self.redis.get(name)
        return float(output) if output else None
    
    def redis_to_map(self, name):
        encoded = redis.get(name)
        if encoded is None:
            return np.full((self.map_width, self.map_height, 1), self.map_base_height, np.uint8)
        else:
            h, w = struct.unpack('>II', encoded[:8])
            received_array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 1)
            #received_array = cv2.cvtColor(received_array,cv2.COLOR_GRAY2RGB)
            
            return received_array


    def angle_and_distance_to_target(self):
        received_target_coords = self.redis.get('target_car_coords')
        
        if received_target_coords is not None:
            target_car_coords = np.array(struct.unpack('%sf' %3, received_target_coords))
            car2target_vector = [target_car_coords[0], target_car_coords[2]]  # Assuming x and z are the ground plane
            
            z_vector = np.array([0, 1])
            angle = np.degrees(np.math.atan2(np.linalg.det([car2target_vector, z_vector]), np.dot(car2target_vector, z_vector)))
            distance = np.linalg.norm(car2target_vector)   
            
            if angle < -180:
                angle = 360 + angle
            if angle > 180:
                angle = 360 - angle  
            
            self.r.psetex('log_target_distance', 1000, distance)
            self.r.psetex('log_target_angle', 1000, angle)

            return angle, distance
        else:
            return False, False

    def compute_path_costs(self):
        self.map = self.redis_to_map("map")
        self.path_costs = [float('inf')] * 11  # Initialize with infinity
        self.path_angle_costs = [[] for _ in range(11)]
        self.path_heights = [[] for _ in range(11)]
        self.angle, self.distance = self.angle_and_distance_to_target()

        if not self.angle and not self.distance:
            self.redis.psetex('path', 1000, -1)
            return

        for path in range(11):
            self.compute_cost_for_path(path)
        
        self.choose_best_path()

    def compute_cost_for_path(self, path):
        if path > 5:
            path_lookup = path - 5
            l = -1
            target_angle = -pc.paths[path_lookup]['target_angle']
            
        else:
            l = 1
            path_lookup = path
            target_angle = pc.paths[path_lookup]['target_angle']
        
        height_difference_cost = 0
        square_to_square_cost = 0
        max_height = 0
        
        angle_cost = m.pow(abs(target_angle - self.angle) * self.angle_deviation_cost_factor, self.angle_deviation_expo)
        for square in range(0, self.square_range):
            #print(path,square)
            
            x0 = int(l * pc.paths[path_lookup]['coords'][square][0] / 10 + self.map_width / 2)
            y0 = self.map_height - int(pc.paths[path_lookup]['coords'][square][1] / 10 + 150)
            x1 = int(l * pc.paths[path_lookup]['coords'][square][2] / 10 + self.map_width / 2)
            y1 = self.map_height - int(pc.paths[path_lookup]['coords'][square][3] / 10 + 150)
            
            x2 = int(l * pc.paths[path_lookup]['coords'][square + 1][0] / 10 + self.map_width / 2)
            y2 = self.map_height - int(pc.paths[path_lookup]['coords'][square + 1][1] / 10 + 150)
            x3 = int(l * pc.paths[path_lookup]['coords'][square + 1][2] / 10 + self.map_width / 2)
            y3 = self.map_height - int(pc.paths[path_lookup]['coords'][square + 1][3] / 10 + 150)
            poly = np.array([[x0,y0],[x1,y1],[x3,y3],[x2,y2]])
            
            poly = poly.reshape((-1, 1, 2))
            
            
            mask = np.zeros(map.shape, dtype=np.uint8)
            cv2.fillPoly(mask,[poly],(255,255,255))
            crop = cv2.bitwise_and(map,mask)
            bg = np.ones_like(crop, np.uint8) * self.map_base_height
            bgmask= cv2.bitwise_not(mask)
            bg = cv2.bitwise_and(bg,bgmask)
            crop = crop + bg
            #print(crop)


            min_height = crop.min() - 100
            max_height_previous_step = max_height
            max_height = crop.max() - 100
            
            self.path_heights[path].append(max_height)

            if square > 3:
                height_difference_cost += (max_height - min_height) / 4
                square_to_square_cost += (abs(max_height - max_height_previous_step)) / 4
            
            else:
                if abs(max_height - max_height_previous_step) > self.max_climb_height:
                    square_to_square_cost += 1000
                else:
                    square_to_square_cost += abs(max_height - max_height_previous_step) * self.square_to_square_cost_factor
        
        total_cost = angle_cost + square_to_square_cost #height_difference_cost not cosnidered right now
        self.path_costs[path] = total_cost

    def main_loop(self):
        while True:
            starttime = time.time()
            self.load_configuration()
            map_data = self.redis_to_map("map")
            # Refactor the code blocks into well-named methods.
            # Example:
            self.compute_path_costs()
            # More refactored code...
            time.sleep(0.1)

if __name__ == "__main__":
    navigator = PathNavigator()
    navigator.main_loop()

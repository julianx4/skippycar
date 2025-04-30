#navigation old.py
import time
import math
import struct
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional
import redis
import curved_paths_coords as pc


@dataclass
class NavigationConfig:
    """Configuration parameters for navigation system"""
    angle_deviation_cost_factor: float = 2.0
    angle_deviation_expo: float = 1.0
    driving_speed: float = 30.0
    max_climb_height: float = 10.0
    min_speed: float = 0.05
    min_speed_increase_factor: float = 1.5
    square_range: int = 6
    obstacle_stop_height: float = 15.0
    target_stop_distance: float = 100.0  # Changed to 100cm (1 meter)
    square_to_square_cost_factor: float = 10.0
    map_base_height: int = 100


class PathCosts:
    """Stores and manages cost calculations for different paths"""
    def __init__(self, num_paths: int):
        self.costs = [float('inf')] * num_paths
        self.heights = [[] for _ in range(num_paths)]
        self.angle_costs = [[] for _ in range(num_paths)]
        
    def get_min_cost(self) -> Tuple[float, int]:
        min_cost = min(self.costs)
        return min_cost, self.costs.index(min_cost)


class NavigationSystem:
    """Main navigation system that handles path planning and control"""
    def __init__(self):
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.config = NavigationConfig()
        self.map_dimensions = (400, 400)  # width, height
        self.in_front_of_car = 0
        self.target_speed = None
        self.in_motion_start = time.time()
        self.car_position = (self.map_dimensions[0] // 2, self.map_dimensions[1] - 250)

    def _publish_navigation_overlay(self, paths: List[dict]) -> None:
        """Publish navigation visualization overlay to Redis"""
        overlay = np.zeros((*self.map_dimensions, 4), dtype=np.uint8)
        
        # Find best path (lowest cost)
        best_path_idx = min(range(len(paths)), key=lambda i: paths[i]['cost'])
        
        for i, path_data in enumerate(paths):
            points = path_data['points']
            cost = path_data['cost']
            
            if len(points) < 2:
                continue
                
            # Normalize cost for visualization (adjust range as needed)
            cost_normalized = max(0.3, min(1.0, 1.0 - (cost / 1000)))
            
            if i == best_path_idx:
                # Best path in green
                color = (0, 255, 0, int(255 * cost_normalized))
                thickness = 2
            else:
                # Other paths in blue/red based on cost
                if cost < 500:  # Adjust threshold as needed
                    color = (50, 50, 200, int(180 * cost_normalized))  # Blue for good paths
                else:
                    color = (200, 50, 50, int(180 * cost_normalized))  # Red for bad paths
                thickness = 1

            # Convert points to numpy array for drawing
            points = np.array(points, dtype=np.int32)
            
            # Draw path segments
            for j in range(len(points) - 1):
                cv2.line(overlay,
                        tuple(points[j]),
                        tuple(points[j + 1]),
                        color,
                        thickness)

        # Send to Redis
        h, w = overlay.shape[:2]
        shape = struct.pack('>II', h, w)
        encoded = shape + overlay.tobytes()
        self.redis_client.psetex('overlay_navigation', 1000, encoded)

    def calculate_path_costs(self, angle: float) -> PathCosts:
        """Calculate costs for all possible paths"""
        path_costs = PathCosts(11)
        map_data = self.get_map_data()
        
        paths_visualization = []  # Store path data for visualization
        
        for path_idx in range(11):
            path_points = self._calculate_single_path_cost(
                path_idx, angle, map_data, path_costs
            )
            
            # Store path data for visualization
            paths_visualization.append({
                'points': path_points,
                'cost': path_costs.costs[path_idx]
            })

        # Publish visualization data
        self._publish_navigation_overlay(paths_visualization)
        
        return path_costs

    def _calculate_single_path_cost(
        self, 
        path_idx: int, 
        target_angle: float, 
        map_data: np.ndarray, 
        path_costs: PathCosts
    ) -> List[Tuple[int, int]]:
        """Calculate cost for a single path and return path points for visualization"""
        path_lookup, direction = self._get_path_parameters(path_idx)
        
        # Initialize cost components and path points
        square_to_square_cost = 0
        max_height = 0
        path_points = []
        
        # Calculate angle deviation cost
        path_angle = -pc.paths[path_lookup]['target_angle'] if direction == -1 else pc.paths[path_lookup]['target_angle']
        angle_cost = math.pow(
            abs(path_angle - target_angle) * self.config.angle_deviation_cost_factor,
            self.config.angle_deviation_expo
        )

        # Calculate costs for each square in the path
        for square in range(self.config.square_range):
            poly = self._get_path_polygon(path_lookup, square, direction)
            square_costs = self._calculate_square_costs(
                poly, map_data, max_height, square
            )
            
            max_height = square_costs['max_height']
            square_to_square_cost += square_costs['cost']
            path_costs.heights[path_idx].append(max_height)
            
            # Store center point of polygon for visualization
            center_point = poly.mean(axis=0).astype(int)
            path_points.append(center_point)

        path_costs.costs[path_idx] = angle_cost + square_to_square_cost
        return path_points

    def get_redis_float(self, name: str, default: float = None) -> float:
        """Retrieve float value from Redis with fallback default"""
        value = self.redis_client.get(name)
        return float(value) if value is not None else default

    def update_config(self) -> None:
        """Update configuration parameters from Redis"""
        self.config.angle_deviation_cost_factor = self.get_redis_float('angle_deviation_cost_factor', 2)
        self.config.driving_speed = self.get_redis_float('driving_speed', 30)
        self.config.max_climb_height = self.get_redis_float('max_climb_height', 10)
        self.config.min_speed = self.get_redis_float('min_speed', 0.05)
        self.config.min_speed_increase_factor = self.get_redis_float('min_speed_increase_factor', 1.5)
        self.config.square_range = int(self.get_redis_float('square_range', 6))
        self.config.obstacle_stop_height = self.get_redis_float('obstacle_stop_height', 15)
        self.config.target_stop_distance = self.get_redis_float('target_stop_distance', 100)
        self.config.square_to_square_cost_factor = self.get_redis_float('square_to_square_cost_factor', 10)
        self.config.map_base_height = int(self.get_redis_float('map_base_height', 100))

    def get_angle_and_distance_to_target(self) -> Tuple[Optional[float], Optional[float]]:
        """Calculate angle and distance to target in car coordinates"""
        target_coords_raw = self.redis_client.get('target_car_coords')
        if target_coords_raw is None:
            return None, None

        target_coords = np.array(struct.unpack('%sf' % 3, target_coords_raw))
        car2target_vector = [target_coords[0], target_coords[2]]
        z_vector = np.array([0, 1])
        
        angle = np.degrees(np.math.atan2(
            np.linalg.det([car2target_vector, z_vector]),
            np.dot(car2target_vector, z_vector)
        ))
        
        # Normalize angle to [-180, 180]
        if angle < -180:
            angle += 360
        if angle > 180:
            angle = 360 - angle

        distance = np.linalg.norm(car2target_vector)
        
        # Update Redis with calculated values
        self.redis_client.psetex('log_target_distance', 1000, distance)
        self.redis_client.psetex('log_target_angle', 1000, angle)
        
        return angle, distance

    def _get_path_parameters(self, path_idx: int) -> Tuple[int, int]:
        """Get path lookup index and direction"""
        if path_idx > 5:
            return path_idx - 5, -1
        return path_idx, 1

    def _get_path_polygon(self, path_lookup: int, square: int, direction: int) -> np.ndarray:
        """Generate polygon coordinates for path segment"""
        coords = pc.paths[path_lookup]['coords']
        
        points = []
        for i in range(2):
            for j in range(2):
                x = direction * coords[square + i][j * 2] / 10 + self.map_dimensions[0] / 2
                y = self.map_dimensions[1] - coords[square + i][j * 2 + 1] / 10 - 300
                points.append([int(x), int(y)])
                
        return np.array(points)

    def _calculate_square_costs(
        self, 
        poly: np.ndarray, 
        map_data: np.ndarray, 
        prev_max_height: float,
        square_idx: int
    ) -> dict:
        """Calculate costs for a single square in the path"""
        mask = np.zeros(map_data.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [poly], (255, 255, 255))
        
        crop = cv2.bitwise_and(map_data, mask)
        bg = np.ones_like(crop, np.uint8) * self.config.map_base_height
        bgmask = cv2.bitwise_not(mask)
        bg = cv2.bitwise_and(bg, bgmask)
        crop = crop + bg

        min_height = crop.min() - 100
        max_height = crop.max() - 100
        
        if square_idx > 3:
            cost = (abs(max_height - prev_max_height)) / 4
        else:
            if abs(max_height - prev_max_height) > self.config.max_climb_height:
                cost = 1000
            else:
                cost = abs(max_height - prev_max_height) * self.config.square_to_square_cost_factor
                
        return {
            'max_height': max_height,
            'cost': cost
        }

    def get_map_data(self) -> np.ndarray:
        """Retrieve and decode map data from Redis"""
        encoded_map = self.redis_client.get("map")
        if encoded_map is None:
            return np.full(self.map_dimensions + (1,), self.config.map_base_height, np.uint8)
            
        h, w = struct.unpack('>II', encoded_map[:8])
        return np.frombuffer(encoded_map, dtype=np.uint8, offset=8).reshape(h, w, 1)

    def update_vehicle_control(self, path_costs: PathCosts) -> None:
        """Update vehicle control based on path costs"""
        min_cost, best_path = path_costs.get_min_cost()
        
        # Store path heights in Redis
        for i, height in enumerate(path_costs.heights[best_path]):
            self.redis_client.psetex(f'square_height{i}', 1000, float(height))
            
        # Store path costs in Redis
        for i, cost in enumerate(path_costs.costs):
            self.redis_client.psetex(f'path_cost{i}', 1000, float(cost))
            self.redis_client.psetex(f'path_angle_cost{i}', 1000, float(cost))

        self.redis_client.psetex('path_min_cost', 1000, str(min_cost))
        
        # Calculate and set steering angle
        if best_path > 5:
            path_lookup = best_path - 5
            steering_angle = -pc.paths[path_lookup]['steering_angle']
        else:
            steering_angle = pc.paths[best_path]['steering_angle']
            
        self.redis_client.psetex('angle', 1000, steering_angle)
        
        # Update path in Redis
        if not any([
            min_cost > 1000,
            self.in_front_of_car > self.config.obstacle_stop_height,
            self.distance < self.config.target_stop_distance
        ]):
            self.redis_client.psetex('path', 1000, best_path)
            self._update_speed_control()

    def _update_speed_control(self) -> None:
        """Update vehicle speed based on current conditions"""
        current_speed = self.get_redis_float('current_speed')
        
        if current_speed is not None and current_speed < self.config.min_speed and \
           time.time() - self.in_motion_start > 2:
            target_speed = self.config.driving_speed * self.config.min_speed_increase_factor
        else:
            target_speed = self.config.driving_speed
            
        self.redis_client.psetex('target_speed', 1000, target_speed)

    def process_navigation_cycle(self) -> None:
        """Process one complete navigation cycle"""
        # Update configuration from Redis
        self.update_config()
        
        # Get current map and obstacle information
        map_data = self.get_map_data()
        crop = map_data[230:241, 188:212]
        self.in_front_of_car = crop.max() - 100
        self.redis_client.psetex('log_in_front_of_car', 1000, float(self.in_front_of_car))
        
        # Get target information
        angle, self.distance = self.get_angle_and_distance_to_target()
        
        # Stop if no target or within stop distance
        if angle is None or self.distance < self.config.target_stop_distance:
            self.redis_client.psetex('path', 1000, -1)
            self.redis_client.psetex('angle', 1000, 0)
            self.redis_client.psetex('target_speed', 1000, 0)
            return
            
        # Calculate path costs and update control
        path_costs = self.calculate_path_costs(angle)
        self.update_vehicle_control(path_costs)

    def run(self) -> None:
        """Main navigation loop"""
        while True:
            # Update navigation status
            self.redis_client.psetex('log_navigation_running', 1000, "on")
            try:
                self.process_navigation_cycle()
                time.sleep(0.1)
            except Exception as e:
                print(f"Error in navigation cycle: {e}")
                time.sleep(1)  # Delay before retry on error


if __name__ == "__main__":
    navigator = NavigationSystem()
    navigator.run()
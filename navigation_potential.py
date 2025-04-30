# navigation_potential.py
import numpy as np
import redis
import struct
import cv2
import time
from dataclasses import dataclass
from typing import Tuple, Optional, List
from config import MapConfig

@dataclass
class NavigationConfig:
    """Configuration for navigation"""
    def __init__(self):
        # Initialize MapConfig for dimensions
        self.map_config = MapConfig()
        self.map_width, self.map_height, self.cm_per_pixel = self.map_config.get_dimensions()
        
        # Navigation parameters
        self.look_ahead_distance = 150  # cm
        self.scan_width = 100  # cm
        self.angle_range = 60  # degrees total scan range (+/- 30 degrees)
        self.num_scan_angles = 7  # Number of angles to check
        self.target_speed = 20.0  # Fixed speed
        self.smoothing_factor = 0.3  # Steering angle smoothing
        
        # Height thresholds
        self.base_height = 100  # Base ground height
        self.max_safe_height = 10  # Maximum traversable height difference in cm

class PathNavigator:
    def __init__(self):
        self.config = NavigationConfig()
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.prev_angle = 0.0
        
        # Car position (fixed, matching sensing_advanced.py)
        self.car_pos = np.array([
            self.config.map_width // 2,
            self.config.map_height - (250 // self.config.cm_per_pixel)
        ])

    def _get_target_coords(self) -> Optional[np.ndarray]:
        """Get target coordinates from Redis"""
        target_data = self.redis_client.get('target_car_coords')
        if target_data:
            coords = np.array(struct.unpack('%sf' % 3, target_data))
            # Convert from meters to pixels (50 pixels per meter at 2cm/pixel)
            map_x = int(coords[0] * 50 + self.config.map_width / 2)
            map_y = int(self.config.map_height - coords[2] * 50 - 75)
            return np.array([map_x, map_y])
        return None

    def _decode_height_map(self, encoded_map: bytes) -> Tuple[np.ndarray, np.ndarray]:
        """Decode combined height and confidence map from Redis"""
        if encoded_map is None:
            return (
                np.full((self.config.map_height, self.config.map_width), self.config.base_height),
                np.zeros((self.config.map_height, self.config.map_width))
            )
            
        h, w = struct.unpack('>II', encoded_map[:8])
        data = np.frombuffer(encoded_map[8:], dtype=np.uint16).reshape(h, w)
        
        height = ((data >> 4) & 0x0FFF).astype(np.int16)
        confidence = (data & 0x0F) / 15.0
        
        return height, confidence

    def _publish_navigation_overlay(self, paths: List[Tuple[np.ndarray, np.ndarray, float]], best_path_idx: int):
        """Publish navigation visualization overlay to Redis"""
        # Create RGBA overlay
        overlay = np.zeros((self.config.map_height, self.config.map_width, 4), dtype=np.uint8)
        
        # Draw all scanned paths
        for i, (x_points, y_points, score) in enumerate(paths):
            if len(x_points) == 0:
                continue
            
            # Normalize score to 0-1 range for color intensity
            score_normalized = max(0.3, min(1.0, (score + 2) / 4))  # Adjust range as needed
            
            if i == best_path_idx:
                # Best path in green
                color = (0, 255, 0, int(255 * score_normalized))
                thickness = 2
            else:
                # Other paths in blue/red based on score
                if score > 0:
                    color = (50, 50, 200, int(180 * score_normalized))  # Blue for good paths
                else:
                    color = (200, 50, 50, int(180 * score_normalized))  # Red for bad paths
                thickness = 1

            # Draw path
            for j in range(len(x_points) - 1):
                cv2.line(overlay,
                        (int(x_points[j]), int(y_points[j])),
                        (int(x_points[j + 1]), int(y_points[j + 1])),
                        color,
                        thickness)

        # Send to Redis
        h, w = overlay.shape[:2]
        shape = struct.pack('>II', h, w)
        encoded = shape + overlay.tobytes()
        self.redis_client.psetex('overlay_navigation', 1000, encoded)

        # Also publish navigation parameters for visualization
        self.redis_client.psetex('nav_look_ahead', 1000, self.config.look_ahead_distance)
        self.redis_client.psetex('nav_scan_width', 1000, self.config.scan_width)

    def find_best_path(self, height_map: np.ndarray, confidence_map: np.ndarray, target_pos: Optional[np.ndarray]) -> float:
        """Find the best steering angle by scanning different angles ahead"""
        # Convert distances from cm to pixels
        look_ahead = int(self.config.look_ahead_distance / self.config.cm_per_pixel)
        scan_width = int(self.config.scan_width / self.config.cm_per_pixel)
        
        # Generate angles to check
        angles = np.linspace(-self.config.angle_range/2, self.config.angle_range/2, self.config.num_scan_angles)
        best_score = float('-inf')
        best_angle = 0
        
        # If we have a target, calculate desired direction
        target_angle = 0
        if target_pos is not None:
            direction = target_pos - self.car_pos
            target_angle = np.degrees(np.arctan2(direction[0], -direction[1]))
        
        # Store all paths for visualization
        paths = []
        
        # Check each potential path
        for angle in angles:
            # Calculate path points
            rad_angle = np.radians(angle)
            end_x = int(self.car_pos[0] + look_ahead * np.sin(rad_angle))
            end_y = int(self.car_pos[1] - look_ahead * np.cos(rad_angle))
            
            # Get points along the path
            num_points = 20
            x_points = np.linspace(self.car_pos[0], end_x, num_points).astype(int)
            y_points = np.linspace(self.car_pos[1], end_y, num_points).astype(int)
            
            # Clip to map boundaries
            mask = (x_points >= 0) & (x_points < self.config.map_width) & \
                   (y_points >= 0) & (y_points < self.config.map_height)
            x_points = x_points[mask]
            y_points = y_points[mask]
            
            if len(x_points) == 0:
                continue
            
            # Calculate path score based on:
            # 1. Height differences (lower is better)
            # 2. Confidence (higher is better)
            # 3. Angle to target (closer to target angle is better)
            height_diffs = np.abs(height_map[y_points, x_points] - self.config.base_height)
            conf_values = confidence_map[y_points, x_points]
            
            # Penalize height differences above max_safe_height
            height_penalty = np.sum(height_diffs > self.config.max_safe_height)
            
            # Calculate average confidence along path
            conf_score = np.mean(conf_values)
            
            # Calculate angle score (if we have a target)
            angle_score = 0
            if target_pos is not None:
                angle_diff = abs(angle - target_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                angle_score = 1.0 - (angle_diff / 180.0)
            
            # Combine scores (adjust weights as needed)
            score = -height_penalty + conf_score + angle_score
            
            # Store path data for visualization
            paths.append((x_points, y_points, score))
            
            if score > best_score:
                best_score = score
                best_angle = angle
                best_path_idx = len(paths) - 1
        
        # Publish visualization data
        self._publish_navigation_overlay(paths, best_path_idx)
        
        # Smooth the steering angle
        best_angle = (1 - self.config.smoothing_factor) * self.prev_angle + \
                    self.config.smoothing_factor * best_angle
        self.prev_angle = best_angle
        
        return best_angle

    def process_navigation_cycle(self):
        """Process one complete navigation cycle"""
        try:
            # Get target position
            target_pos = self._get_target_coords()
            
            # If no target, stop the car
            if target_pos is None:
                self.redis_client.psetex('angle', 300, 0)
                self.redis_client.psetex('target_speed', 300, 0)
                return
            
            # Get and decode map data
            encoded_map = self.redis_client.get('raw_map')
            height_map, confidence_map = self._decode_height_map(encoded_map)
            
            # Calculate distance to target
            distance = np.linalg.norm(target_pos - self.car_pos)
            
            # Stop if within 1 meter (100cm)
            if distance < 100 / self.config.cm_per_pixel:
                self.redis_client.psetex('angle', 300, 0)
                self.redis_client.psetex('target_speed', 300, 0)
                return
            
            # Find best steering angle
            angle = self.find_best_path(height_map, confidence_map, target_pos)
            
            # Send commands to Redis
            self.redis_client.psetex('angle', 300, angle)
            self.redis_client.psetex('target_speed', 300, self.config.target_speed)
            
        except Exception as e:
            print(f"Error in navigation cycle: {e}")
            self.redis_client.set('angle', 0)
            self.redis_client.set('target_speed', 0)

    def run(self):
        """Main navigation loop"""
        while True:
            try:
                self.process_navigation_cycle()
                time.sleep(0.05)  # 20Hz update rate
            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(1)

if __name__ == "__main__":
    navigator = PathNavigator()
    navigator.run()
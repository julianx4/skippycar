# navigation_potential.py
import numpy as np
import redis
import struct
import cv2
import time
from dataclasses import dataclass
from typing import Tuple, Optional

@dataclass
class NavigationConfig:
    """Simplified configuration for potential field navigation"""
    # Map dimensions (matching sensing_advanced.py)
    map_width: int = 400
    map_height: int = 400
    map_base_height: int = 100
    cm_per_pixel: float = 2.0
    
    # Force field parameters
    attract_gain: float = 1.5         # Strong attraction to target
    repel_gain: float = 0.2          # Gentle lateral repulsion
    repel_threshold: float = 35.0    # Reaction distance to obstacles
    target_stop_distance: float = 50.0  # Distance to stop from target (in pixels)
    max_force: float = 3.0           # Maximum total force
    smoothing_factor: float = 0.2    # Smoothing for force changes
    
    # Vehicle parameters (in cm)
    vehicle_width: float = 25.0
    vehicle_length: float = 55.0
    min_turning_radius: float = 60.0

class PotentialFieldNavigation:
    def __init__(self):
        self.config = NavigationConfig()
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
        # Initialize force vectors
        self.attract_force = np.zeros(2)
        self.repel_force = np.zeros(2)
        self.total_force = np.zeros(2)
        
        # Add steering history for additional smoothing
        self.prev_angle = 0.0
        
        # Car position (fixed, matching sensing_advanced.py)
        self.car_pos = np.array([
            self.config.map_width // 2,
            self.config.map_height - (250 // self.config.cm_per_pixel)
        ])
        
        # Set initial gear
        self.redis_client.set('gear', 1)

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
                np.full((self.config.map_height, self.config.map_width), self.config.map_base_height),
                np.zeros((self.config.map_height, self.config.map_width))
            )
            
        h, w = struct.unpack('>II', encoded_map[:8])
        data = np.frombuffer(encoded_map[8:], dtype=np.uint16).reshape(h, w)
        
        height = ((data >> 4) & 0x0FFF).astype(np.int16)
        confidence = (data & 0x0F) / 15.0
        
        return height, confidence

    def calculate_attractive_force(self, target_pos: np.ndarray) -> np.ndarray:
        """Calculate attractive force toward target"""
        direction = target_pos - self.car_pos
        distance = np.linalg.norm(direction)
        
        # No attraction when we're at the target stop distance
        if distance < self.config.target_stop_distance:
            return np.zeros(2)
            
        # Scale attraction smoothly as we approach target
        distance_factor = min(1.0, (distance - self.config.target_stop_distance) / 50.0)
        force = direction / distance * self.config.attract_gain * distance_factor
        
        force_magnitude = np.linalg.norm(force)
        if force_magnitude > self.config.max_force:
            force = force * self.config.max_force / force_magnitude
            
        return force

    def calculate_repulsive_force(self, height_map: np.ndarray, confidence_map: np.ndarray) -> np.ndarray:
        """Calculate purely lateral repulsive forces to push car sideways away from obstacles"""
        # Create observation window in front of car
        window_width = int(100 / self.config.cm_per_pixel)  # 100cm wide
        window_height = int(150 / self.config.cm_per_pixel) # 150cm deep
        
        x_start = max(0, int(self.car_pos[0] - window_width // 2))
        x_end = min(self.config.map_width, int(self.car_pos[0] + window_width // 2))
        y_start = max(0, int(self.car_pos[1] - window_height))
        y_end = int(self.car_pos[1])
        
        # Get window data
        height_window = height_map[y_start:y_end, x_start:x_end]
        conf_window = confidence_map[y_start:y_end, x_start:x_end]
        
        # Find obstacles
        height_diff = height_window - self.config.map_base_height
        obstacle_mask = (height_diff > 10) & (conf_window > 0.3)
        
        if not np.any(obstacle_mask):
            return np.zeros(2)
            
        # Get coordinates of all obstacle points
        y_coords, x_coords = np.nonzero(obstacle_mask)
        
        lateral_force = 0.0  # Will be positive for rightward push, negative for leftward
        
        # Window center x-coordinate (relative to window)
        center_x = window_width // 2
        
        # Calculate lateral push from each obstacle
        for x, y in zip(x_coords, y_coords):
            # Calculate how far the obstacle is to left or right of center
            # Negative means obstacle is to the left, positive means to the right
            x_offset = x - center_x
            
            # Distance from car (using y coordinate since this is in window space)
            distance = obstacle_mask.shape[0] - y  # Convert y to distance (higher y = closer)
            
            if distance < self.config.repel_threshold / self.config.cm_per_pixel:
                # Push away from the obstacle
                # If obstacle is on left (negative x_offset), push right (positive force)
                # If obstacle is on right (positive x_offset), push left (negative force)
                push_strength = self.config.repel_gain * (1.0 - distance / (self.config.repel_threshold / self.config.cm_per_pixel))
                lateral_force -= np.sign(x_offset) * push_strength  # Note the negative sign
        
        # Create repulsion vector with only lateral (x) component
        total_repulsion = np.array([lateral_force, 0.0])
        
        # Limit maximum repulsion
        force_magnitude = np.linalg.norm(total_repulsion)
        if force_magnitude > self.config.max_force:
            total_repulsion = total_repulsion * self.config.max_force / force_magnitude
            
        return total_repulsion

    def create_force_visualization(self, target_pos: Optional[np.ndarray]) -> np.ndarray:
        """Create visualization overlay showing forces"""
        overlay = np.zeros((self.config.map_height, self.config.map_width, 4), dtype=np.uint8)
        
        # Draw attractive force in green (points toward target)
        if target_pos is not None:
            cv2.arrowedLine(
                overlay,
                tuple(map(int, self.car_pos)),
                tuple(map(int, self.car_pos + self.attract_force * 100)),
                (0, 255, 0, 255), 2
            )
        
        # Draw repulsive force in red (points away from obstacles)
        cv2.arrowedLine(
            overlay,
            tuple(map(int, self.car_pos)),
            tuple(map(int, self.car_pos + self.repel_force * 100)),
            (0, 0, 255, 255), 2
        )
        
        # Draw total force in blue (combination of attract and repel)
        cv2.arrowedLine(
            overlay,
            tuple(map(int, self.car_pos)),
            tuple(map(int, self.car_pos + self.total_force * 100)),
            (255, 0, 0, 255), 2
        )
        
        return overlay

    def calculate_steering_output(self) -> Tuple[float, float]:
        """Convert force vector to angle and target_speed commands"""
        force_magnitude = np.linalg.norm(self.total_force)
        
        # Get current target distance if available
        target_pos = self._get_target_coords()
        if target_pos is not None:
            distance_to_target = np.linalg.norm(target_pos - self.car_pos)
            if distance_to_target < self.config.target_stop_distance:
                return 0.0, 0.0  # Stop when we're close enough to target
        
        # If no meaningful force or no target, don't move
        if force_magnitude < 1e-6:
            return 0.0, 0.0
            
        # Calculate steering angle
        force_angle = np.arctan2(self.total_force[0], -self.total_force[1])
        angle = np.clip(np.degrees(force_angle), -55.0, 55.0)
        
        # Constant speed of 15
        target_speed = 15.0
            
        return angle, target_speed

    def process_navigation_cycle(self):
        """Process one complete navigation cycle"""
        try:
            # Record that navigation is running
            self.redis_client.psetex('log_navigation_running', 1000, 'on')
            
            # Get target position
            target_pos = self._get_target_coords()
            if target_pos is None:
                print("No target visible")
                self.redis_client.set('angle', 0)
                self.redis_client.set('target_speed', 0)
                return
                
            # Get and decode map data
            encoded_map = self.redis_client.get('raw_map')
            height_map, confidence_map = self._decode_height_map(encoded_map)
            
            # Calculate forces
            self.attract_force = self.calculate_attractive_force(target_pos)
            self.repel_force = self.calculate_repulsive_force(height_map, confidence_map)
            
            # Combine forces with smoothing
            new_total = self.attract_force + self.repel_force
            prev_magnitude = np.linalg.norm(self.total_force)
            self.total_force = (
                self.total_force * (1 - self.config.smoothing_factor) +
                new_total * self.config.smoothing_factor
            )
            
            # Print force vectors for debugging
            print(f"Forces - Attract: [{self.attract_force[0]:.2f}, {self.attract_force[1]:.2f}], " 
                  f"Repel: [{self.repel_force[0]:.2f}, {self.repel_force[1]:.2f}], "
                  f"Total: [{self.total_force[0]:.2f}, {self.total_force[1]:.2f}]")
            
            # Calculate steering output
            angle, target_speed = self.calculate_steering_output()
            
            # Send commands to Redis with 300ms timeout
            self.redis_client.psetex('angle', 300, angle)
            self.redis_client.psetex('target_speed', 300, target_speed)
            
            # Create and send visualization
            force_overlay = self.create_force_visualization(target_pos)
            h, w = force_overlay.shape[:2]
            encoded_overlay = struct.pack('>II', h, w) + force_overlay.tobytes()
            self.redis_client.set('overlay_forces', encoded_overlay)
            
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
    navigator = PotentialFieldNavigation()
    navigator.run()
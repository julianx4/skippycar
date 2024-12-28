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
    attract_gain: float = 1.0          # Gain for attractive force
    repel_gain: float = 5.0           # Gain for repulsive force
    repel_threshold: float = 50.0     # Distance threshold for repulsion (in cm)
    max_force: float = 10.0           # Maximum force magnitude
    smoothing_factor: float = 0.3     # Force smoothing between updates
    
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
        
        # Extract height (12 bits) and confidence (4 bits)
        height = ((data >> 4) & 0x0FFF).astype(np.int16)
        confidence = (data & 0x0F) / 15.0  # Normalize confidence to 0-1
        
        return height, confidence

    def calculate_attractive_force(self, target_pos: np.ndarray) -> np.ndarray:
        """Calculate attractive force toward target"""
        direction = target_pos - self.car_pos
        distance = np.linalg.norm(direction)
        
        # Stop if within 1 meter (50 pixels) of target
        if distance < 50:
            return np.zeros(2)
            
        # Normalize and scale by distance
        force = direction / distance * self.config.attract_gain
        
        # Limit force magnitude
        force_magnitude = np.linalg.norm(force)
        if force_magnitude > self.config.max_force:
            force = force * self.config.max_force / force_magnitude
            
        return force

    def calculate_repulsive_force(self, height_map: np.ndarray, confidence_map: np.ndarray) -> np.ndarray:
        """Calculate repulsive force from obstacles and unobserved areas"""
        # Create observation window in front of car
        window_width = int(100 / self.config.cm_per_pixel)  # 100cm wide
        window_height = int(150 / self.config.cm_per_pixel) # 150cm deep
        
        # Window boundaries
        x_start = max(0, int(self.car_pos[0] - window_width // 2))
        x_end = min(self.config.map_width, int(self.car_pos[0] + window_width // 2))
        y_start = max(0, int(self.car_pos[1] - window_height))
        y_end = int(self.car_pos[1])
        
        # Extract window
        height_window = height_map[y_start:y_end, x_start:x_end]
        conf_window = confidence_map[y_start:y_end, x_start:x_end]
        
        total_repulsion = np.zeros(2)
        
        # 1. Repulsion from obstacles
        height_diff = height_window - self.config.map_base_height
        obstacle_mask = (height_diff > 10) & (conf_window > 0.3)  # 10cm threshold
        
        # Calculate repulsive force from each obstacle point
        y_coords, x_coords = np.nonzero(obstacle_mask)
        
        for x, y in zip(x_coords, y_coords):
            # Convert to map coordinates
            obstacle_pos = np.array([x + x_start, y + y_start])
            
            # Vector from obstacle to car
            direction = self.car_pos - obstacle_pos
            distance = np.linalg.norm(direction)
            
            if distance < self.config.repel_threshold:
                # Force magnitude increases as distance decreases
                magnitude = self.config.repel_gain * (
                    1.0/distance - 1.0/self.config.repel_threshold
                ) / (distance**2)
                
                # Add confidence-weighted repulsion
                total_repulsion += direction / distance * magnitude * conf_window[y, x]
        
        # 2. Repulsion from unobserved areas (low confidence)
        unobserved_mask = conf_window < 0.3  # Consider areas with less than 30% confidence as unobserved
        y_coords, x_coords = np.nonzero(unobserved_mask)
        
        unobserved_repulsion = np.zeros(2)
        for x, y in zip(x_coords, y_coords):
            # Convert to map coordinates
            point_pos = np.array([x + x_start, y + y_start])
            
            # Vector from unobserved point to car
            direction = self.car_pos - point_pos
            distance = np.linalg.norm(direction)
            
            if distance < self.config.repel_threshold:
                # Strong repulsion from unobserved areas
                magnitude = self.config.repel_gain * 2.0 * (  # Double the repulsion for unobserved areas
                    1.0/distance - 1.0/self.config.repel_threshold
                ) / (distance**2)
                
                unobserved_repulsion += direction / distance * magnitude * (1.0 - conf_window[y, x])
        
        total_repulsion += unobserved_repulsion
        
        # Limit total force magnitude
        force_magnitude = np.linalg.norm(total_repulsion)
        if force_magnitude > self.config.max_force:
            total_repulsion = total_repulsion * self.config.max_force / force_magnitude
            
        return total_repulsion

    def create_force_visualization(self, target_pos: Optional[np.ndarray]) -> np.ndarray:
        """Create visualization overlay showing forces"""
        overlay = np.zeros((self.config.map_height, self.config.map_width, 4), dtype=np.uint8)
        
        # Draw attractive force in green
        if target_pos is not None:
            cv2.arrowedLine(
                overlay,
                tuple(map(int, self.car_pos)),
                tuple(map(int, self.car_pos + self.attract_force * 100)),
                (0, 255, 0, 255), 2
            )
        
        # Draw repulsive force in red
        cv2.arrowedLine(
            overlay,
            tuple(map(int, self.car_pos)),
            tuple(map(int, self.car_pos + self.repel_force * 100)),
            (0, 0, 255, 255), 2
        )
        
        # Draw total force in blue
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
        
        # If there's any meaningful force, we should be moving
        if force_magnitude < 1e-6:
            return 0.0, 0.0
            
        # If we get here, we should definitely move - calculate steering angle
        force_angle = np.arctan2(self.total_force[0], -self.total_force[1])
        angle = np.clip(np.degrees(force_angle), -55.0, 55.0)
        
        # Speed is always between 15 and 25 if we're moving at all
        # Only reduce speed for very sharp turns
        if abs(angle) > 45:  # Only reduce speed in very sharp turns
            target_speed = 23.0
        else:
            target_speed = 23.0
            
        return angle, target_speed
        
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
            
            # Calculate steering output
            angle, target_speed = self.calculate_steering_output()
            print(f"Force magnitude: {np.linalg.norm(self.total_force):.2f}, Angle: {angle:.1f}, Speed: {target_speed:.1f}")
            
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
                time.sleep(0.02)  # 10Hz update rate
            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(1)

if __name__ == "__main__":
    navigator = PotentialFieldNavigation()
    navigator.run()
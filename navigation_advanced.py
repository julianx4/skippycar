import time
import math
import struct
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import redis


@dataclass
class NavigationConfig:
    """Configuration parameters for navigation system"""
    # Map dimensions
    map_width: int = 400
    map_height: int = 400
    map_base_height: int = 100
    
    # Terrain analysis parameters
    max_slope_degrees: float = 15.0  # Maximum traversable slope
    cell_size_cm: float = 2.5  # Each cell represents 2.5cm
    min_confidence: float = 0.3  # Minimum confidence to consider cell data valid
    
    # Navigation parameters
    planning_horizon_m: float = 3.0  # How far ahead to plan in meters
    path_resolution_cm: float = 10.0  # Resolution for path planning
    obstacle_margin_cm: float = 20.0  # Safety margin around obstacles
    
    # Vehicle parameters
    min_turning_radius_cm: float = 60.0  # Minimum turning radius
    vehicle_width_cm: float = 30.0  # Vehicle width for collision checking
    vehicle_length_cm: float = 45.0  # Vehicle length for collision checking
    
    def __post_init__(self):
        """Convert some parameters to grid cells"""
        self.planning_horizon_cells = int(self.planning_horizon_m * 100 / self.cell_size_cm)
        self.path_resolution_cells = int(self.path_resolution_cm / self.cell_size_cm)
        self.obstacle_margin_cells = int(self.obstacle_margin_cm / self.cell_size_cm)
        self.min_turning_radius_cells = int(self.min_turning_radius_cm / self.cell_size_cm)

class TerrainAnalyzer:
    """Analyzes terrain features from map data"""
    def __init__(self, config):
        self.config = config
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
        # Kernels for gradient computation (Sobel operators)
        self.gradient_kernel_x = np.array([[-1, 0, 1],
                                         [-2, 0, 2],
                                         [-1, 0, 1]]) * (1.0 / 8.0)
        self.gradient_kernel_y = self.gradient_kernel_x.T
        
        # Initialize visualization colors (BGR format)
        self.slope_colormap = cv2.applyColorMap(
            np.arange(256, dtype=np.uint8), 
            cv2.COLORMAP_JET
        )

    def get_map_data(self) -> Dict[str, np.ndarray]:
            """Retrieve all map data from Redis"""
            maps = {}
            map_keys = {
                'map': 'raw_height_map',
                'map_confidence': 'raw_confidence_map',
                'map_occupancy': 'raw_occupancy_map'
            }
            
            for local_name, redis_name in map_keys.items():
                encoded_map = self.redis_client.get(redis_name)
                if encoded_map is None:
                    shape = (self.config.map_height, self.config.map_width)
                    maps[local_name] = np.full(shape, self.config.map_base_height 
                                            if local_name == 'map' else 0, np.uint8)
                else:
                    h, w = struct.unpack('>II', encoded_map[:8])
                    maps[local_name] = np.frombuffer(encoded_map, dtype=np.uint8, 
                                                offset=8).reshape(h, w)
            return maps
    
    def compute_slope_map(self, height_map: np.ndarray) -> np.ndarray:
        """
        Compute terrain slopes using gradient operators
        Returns slope angles in degrees
        """
        # Convert height map to float and normalize
        height_meters = (height_map.astype(float) - self.config.map_base_height) / 100.0
        
        # Compute gradients (change in height per cell)
        dx = cv2.filter2D(height_meters, -1, self.gradient_kernel_x)
        dy = cv2.filter2D(height_meters, -1, self.gradient_kernel_y)
        
        # Convert to real-world units (cell_size is in cm)
        dx = dx / (self.config.cell_size_cm / 100.0)  # Convert to meters
        dy = dy / (self.config.cell_size_cm / 100.0)
        
        # Calculate slope angles in degrees
        slopes = np.degrees(np.arctan(np.sqrt(dx**2 + dy**2)))
        return slopes

    def create_slope_overlay(self, slope_map: np.ndarray) -> np.ndarray:
        """Create colored slope visualization"""
        # Normalize slopes to 0-255 range for visualization
        slope_normalized = np.clip(slope_map * (255.0 / self.config.max_slope_degrees), 0, 255)
        slope_normalized = slope_normalized.astype(np.uint8)
        
        # Create colored overlay using jet colormap
        overlay = np.zeros((slope_map.shape[0], slope_map.shape[1], 4), dtype=np.uint8)
        colored_slopes = self.slope_colormap[slope_normalized]
        
        # Set alpha based on slope severity
        alpha = slope_normalized  # More severe slopes are more opaque
        
        # Combine color and alpha
        overlay[..., :3] = colored_slopes
        overlay[..., 3] = alpha
        
        return overlay

    def identify_obstacles(self, height_map: np.ndarray, slope_map: np.ndarray) -> np.ndarray:
        """
        Identify obstacles based on height differentials and slopes
        Returns binary obstacle map
        """
        # 1. Identify areas with excessive slope
        slope_obstacles = slope_map > self.config.max_slope_degrees
        
        # 2. Identify sudden height changes
        height_diff_kernel = np.array([[-1, -1, -1],
                                     [-1,  8, -1],
                                     [-1, -1, -1]]) / 8.0
        height_diffs = cv2.filter2D(height_map.astype(float), -1, height_diff_kernel)
        height_threshold = self.config.max_slope_degrees * (self.config.cell_size_cm / 100.0)
        height_obstacles = np.abs(height_diffs) > height_threshold
        
        return np.logical_or(slope_obstacles, height_obstacles)

    def create_obstacle_overlay(self, obstacle_map: np.ndarray) -> np.ndarray:
        """Create colored obstacle visualization"""
        overlay = np.zeros((*obstacle_map.shape, 4), dtype=np.uint8)
        
        # Set obstacle areas to red with 50% opacity
        overlay[obstacle_map, :3] = [0, 0, 255]  # Red in BGR
        overlay[obstacle_map, 3] = 128  # 50% opacity
        
        return overlay

    def analyze_terrain(self) -> None:
        """
        Main terrain analysis function
        Creates and publishes visualization overlays
        """
        # Get raw map data
        height_map, confidence_map, occupancy_map = self.get_map_data()
        if height_map is None:
            return
            
        # Compute slope map
        slope_map = self.compute_slope_map(height_map)
        
        # Identify obstacles
        obstacle_map = self.identify_obstacles(height_map, slope_map)
        
        # Create visualization overlays
        slope_overlay = self.create_slope_overlay(slope_map)
        obstacle_overlay = self.create_obstacle_overlay(obstacle_map)
        
        # Send overlays to Redis
        self._send_overlay_to_redis('overlay_slopes', slope_overlay)
        self._send_overlay_to_redis('overlay_obstacles', obstacle_overlay)
        
        # Store analysis results for path planning
        self.slope_map = slope_map
        self.obstacle_map = obstacle_map

    def _send_overlay_to_redis(self, name: str, overlay: np.ndarray) -> None:
        """Helper to send overlay images to Redis"""
        h, w = overlay.shape[:2]
        shape = struct.pack('>II', h, w)
        encoded = shape + overlay.tobytes()
        self.redis_client.set(name, encoded)

class PathPlanner:
    """Plans optimal path considering terrain features"""
    def __init__(self, config: NavigationConfig):
        self.config = config
        
    def create_cost_map(self, 
                       slope_map: np.ndarray, 
                       obstacle_map: np.ndarray, 
                       confidence_map: np.ndarray) -> np.ndarray:
        """
        Create a cost map for path planning based on terrain features
        """
        # Initialize cost map
        cost_map = np.ones_like(slope_map, dtype=float) * np.inf
        
        # Basic cost is proportional to slope
        valid_cells = ~obstacle_map
        cost_map[valid_cells] = 1.0 + (slope_map[valid_cells] / self.config.max_slope_degrees)
        
        # Increase cost for low confidence areas
        confidence_normalized = confidence_map.astype(float) / 255.0
        confidence_factor = 1.0 + (1.0 - confidence_normalized) * 2.0
        cost_map *= confidence_factor
        
        # Set obstacle costs to infinity
        cost_map[obstacle_map] = np.inf
        
        return cost_map


class AdvancedNavigationSystem:
    """Main navigation system using terrain analysis and advanced path planning"""
    def __init__(self):
        self.config = NavigationConfig()
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.terrain_analyzer = TerrainAnalyzer(self.config)
        self.path_planner = PathPlanner(self.config)
        
    def get_map_data(self) -> Dict[str, np.ndarray]:
        """Retrieve all map data from Redis"""
        maps = {}
        for map_name in ['map', 'map_confidence', 'map_occupancy']:
            encoded_map = self.redis_client.get(map_name)
            if encoded_map is None:
                shape = (self.config.map_height, self.config.map_width)
                maps[map_name] = np.full(shape, self.config.map_base_height 
                                       if map_name == 'map' else 0, np.uint8)
            else:
                h, w = struct.unpack('>II', encoded_map[:8])
                maps[map_name] = np.frombuffer(encoded_map, dtype=np.uint8, 
                                             offset=8).reshape(h, w)
        return maps

    def process_navigation_cycle(self):
            """Process one complete navigation cycle"""
            # Let terrain analyzer handle all terrain analysis and visualization
            self.terrain_analyzer.analyze_terrain()
            
            # Update navigation status
            self.redis_client.psetex('log_navigation_running', 1000, "on")

    def run(self):
        """Main navigation loop"""
        while True:
            try:
                self.process_navigation_cycle()
                time.sleep(0.1)
            except Exception as e:
                print(f"Error in navigation cycle: {e}")
                time.sleep(1)


if __name__ == "__main__":
    navigator = AdvancedNavigationSystem()
    navigator.run()
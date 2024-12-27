import time
import math
import struct
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import redis

class RRTNode:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


@dataclass
class NavigationConfig:
    """Configuration parameters for navigation system"""
    # Map dimensions - align with new resolution
    map_width: int = 400    # Changed from 800
    map_height: int = 400   # Changed from 800
    map_base_height: int = 100
    
    # Terrain analysis parameters
    max_slope_degrees: float = 35.0
    cell_size_cm: float = 2.0  # Changed to match new resolution (2cm per pixel)
    min_confidence: float = 0.3

    # Navigation parameters (adjusted for new resolution)
    planning_horizon_m: float = 3.0
    path_resolution_cm: float = 10.0
    obstacle_margin_cm: float = 20.0
    
    # Vehicle parameters (these stay the same in cm)
    min_turning_radius_cm: float = 60.0
    vehicle_width_cm: float = 25.0
    vehicle_length_cm: float = 55.0

class TerrainAnalyzer:
    """Analyzes terrain features from map data"""
    def __init__(self, config):
        self.config = config
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
        # Kernels for gradient computation
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
        """Retrieve all map data from Redis with size verification"""
        maps = {}
        map_keys = {
            'map': 'raw_height_map',
            'map_confidence': 'raw_confidence_map',
            'map_occupancy': 'raw_occupancy_map'
        }
        
        expected_shape = (self.config.map_height, self.config.map_width)
        
        for local_name, redis_name in map_keys.items():
            encoded_map = self.redis_client.get(redis_name)
            if encoded_map is None:
                maps[local_name] = np.full(expected_shape, 
                                        self.config.map_base_height if local_name == 'map' else 0, 
                                        np.uint8)
            else:
                try:
                    h, w = struct.unpack('>II', encoded_map[:8])
                    # Verify dimensions match our configuration
                    if (h, w) != expected_shape:
                        print(f"Warning: {redis_name} has wrong dimensions ({h}x{w}), expected {expected_shape}")
                        maps[local_name] = np.full(expected_shape,
                                                self.config.map_base_height if local_name == 'map' else 0,
                                                np.uint8)
                        continue
                    
                    maps[local_name] = np.frombuffer(encoded_map, dtype=np.uint8,
                                                offset=8).reshape(h, w)
                except Exception as e:
                    print(f"Error decoding {redis_name}: {e}")
                    maps[local_name] = np.full(expected_shape,
                                            self.config.map_base_height if local_name == 'map' else 0,
                                            np.uint8)
        return maps

    def compute_slope_map(self, height_map: np.ndarray, confidence_map: np.ndarray) -> np.ndarray:
        """
        Compute terrain slopes with debugging
        """
        print("\n=== Slope Computation Debug ===")
        
        # Convert to meters
        height_meters = (height_map.astype(float) - self.config.map_base_height) / 100.0
        confidence_normalized = confidence_map.astype(float) / 255.0
        
        # Calculate height differences using a larger kernel
        height_kernel = np.array([[-1, -1, -1],
                                [-1,  8, -1],
                                [-1, -1, -1]]) / 8.0
        height_diffs = cv2.filter2D(height_meters, -1, height_kernel)
        
        # Calculate slopes
        smoothed_heights = cv2.GaussianBlur(height_meters, (3, 3), 0)
        dx = cv2.filter2D(smoothed_heights, -1, self.gradient_kernel_x)
        dy = cv2.filter2D(smoothed_heights, -1, self.gradient_kernel_y)
        slopes = np.degrees(np.arctan(np.sqrt(dx**2 + dy**2)))
        
        print(f"Height range: {np.min(height_meters):.2f}m to {np.max(height_meters):.2f}m")
        print(f"Slope range: {np.min(slopes):.2f}° to {np.max(slopes):.2f}°")
        print(f"Points with slope > {self.config.max_slope_degrees}°: {np.sum(slopes > self.config.max_slope_degrees)}")
        
        return slopes

    def create_slope_overlay(self, slope_map: np.ndarray, confidence_map: np.ndarray) -> np.ndarray:
        """Create colored slope visualization with confidence consideration"""
        confidence_normalized = confidence_map.astype(float) / 255.0
        
        # Apply non-linear scaling to emphasize meaningful slopes
        max_slope = self.config.max_slope_degrees
        slope_scaled = np.clip(slope_map * (1.0 + (1.0 - confidence_normalized)), 0, max_slope)
        slope_normalized = np.clip((slope_scaled / max_slope) ** 0.5 * 255, 0, 255).astype(np.uint8)
        
        # Create colored overlay with confidence-based alpha
        colored_slopes = cv2.applyColorMap(slope_normalized, cv2.COLORMAP_JET)
        
        overlay = np.zeros((*slope_map.shape, 4), dtype=np.uint8)
        overlay[..., :3] = colored_slopes
        overlay[..., 3] = (confidence_normalized * 255).astype(np.uint8)
        
        return overlay

    def identify_obstacles(self, height_map: np.ndarray, slope_map: np.ndarray, 
                        confidence_map: np.ndarray) -> np.ndarray:
        """
        Identify obstacles with improved height threshold and blind spot handling
        """
        print("\n=== Obstacle Detection Debug ===")
        
        confidence_normalized = confidence_map.astype(float) / 255.0
        confidence_mask = confidence_normalized > self.config.min_confidence
        
        # 1. Slope-based obstacles (keep as is since it's working well)
        slope_obstacles = slope_map > self.config.max_slope_degrees
        
        # 2. Height differential obstacles with more lenient threshold
        height_diff_kernel = np.array([[-1, -1, -1],
                                    [-1,  8, -1],
                                    [-1, -1, -1]]) / 8.0
        height_diffs = cv2.filter2D(height_map.astype(float), -1, height_diff_kernel)
        
        # Increase height threshold significantly
        height_threshold = 5.0  # Allow up to 5cm height difference between adjacent cells
        height_obstacles = np.abs(height_diffs) > height_threshold
        
        # Combine obstacles and apply confidence
        obstacles = np.logical_or(slope_obstacles, height_obstacles)
        obstacles[~confidence_mask] = False
        
        # Mark blind spot as traversable
        car_x, car_y = 100, 125  # Was 200, 250 (halved for new resolution)
        blind_spot_width = 20    # Was 40 (halved)
        blind_spot_depth = 8     # Was 16 (halved)
        
        # Create blind spot mask
        y_start = car_y - blind_spot_depth
        x_start = car_x - blind_spot_width // 2
        x_end = car_x + blind_spot_width // 2
        
        # Ensure we're within map bounds
        y_start = max(0, y_start)
        x_start = max(0, x_start)
        x_end = min(obstacles.shape[1], x_end)
        
        # Mark blind spot as non-obstacle
        obstacles[y_start:car_y, x_start:x_end] = False
        
        print(f"Total obstacle points: {np.sum(obstacles)}")
        print(f"Blind spot area marked as traversable: {blind_spot_width}x{blind_spot_depth} pixels")
        
        return obstacles


    def create_obstacle_overlay(self, obstacle_map: np.ndarray, 
                              confidence_map: np.ndarray) -> np.ndarray:
        """Create colored obstacle visualization"""
        confidence_normalized = confidence_map.astype(float) / 255.0
        
        overlay = np.zeros((*obstacle_map.shape, 4), dtype=np.uint8)
        
        # Set obstacle areas to red with confidence-based opacity
        overlay[obstacle_map, :3] = [0, 0, 255]  # Red in BGR
        overlay[obstacle_map, 3] = (confidence_normalized[obstacle_map] * 255).astype(np.uint8)
        
        return overlay

    def analyze_terrain(self) -> None:
        """Main terrain analysis function"""
        # Get raw map data
        maps = self.get_map_data()
        if not maps:
            return
                
        # Compute slope map with confidence
        slope_map = self.compute_slope_map(maps['map'], maps['map_confidence'])
        
        # Identify obstacles with confidence
        obstacle_map = self.identify_obstacles(maps['map'], slope_map, maps['map_confidence'])
        
        # Create visualization overlays
        slope_overlay = self.create_slope_overlay(slope_map, maps['map_confidence'])
        obstacle_overlay = self.create_obstacle_overlay(obstacle_map, maps['map_confidence'])
        
        # Send overlays to Redis
        self._send_overlay_to_redis('overlay_slopes', slope_overlay)
        self._send_overlay_to_redis('overlay_obstacles', obstacle_overlay)

    def _send_overlay_to_redis(self, name: str, overlay: np.ndarray) -> None:
        """Helper to send overlay images to Redis with expiry"""
        try:
            h, w = overlay.shape[:2]
            shape = struct.pack('>II', h, w)
            encoded = shape + overlay.tobytes()
            # Set data with 2-second expiry
            self.redis_client.psetex(name, 2000, encoded)
        except Exception as e:
            print(f"Error sending overlay {name} to Redis: {e}")

class PathPlanner:
    def __init__(self, config: NavigationConfig):
        self.config = config
        self.nodes = []
        self.step_size = 20  # pixels
        self.max_iterations = 1000
        self.goal_sample_rate = 0.2
        

    def create_cost_map(self, slope_map: np.ndarray, obstacle_map: np.ndarray, 
                    confidence_map: np.ndarray) -> np.ndarray:
        """Create cost map with guaranteed valid region around car"""
        print("\n=== Cost Map Creation Debug ===")
        
        # Initialize cost map
        cost_map = np.ones_like(slope_map, dtype=float) * np.inf
        
        # Convert confidence to 0-1 range
        confidence_normalized = confidence_map.astype(float) / 255.0
        
        # Create observation mask with lower threshold
        observed_mask = confidence_normalized > 0.1
        
        # Create valid area mask
        valid_mask = ~obstacle_map & observed_mask
        
        # Create guaranteed valid region around car
        car_pos = (100, 125)  # Was (200, 250)
        safe_radius = 15      # Was 30
        
        # Calculate bounds for the car area
        y_start = max(0, car_pos[1] - safe_radius)
        y_end = min(valid_mask.shape[0], car_pos[1] + safe_radius + 1)
        x_start = max(0, car_pos[0] - safe_radius)
        x_end = min(valid_mask.shape[1], car_pos[0] + safe_radius + 1)
        
        # Create the circular mask only for the region we need
        y_size = y_end - y_start
        x_size = x_end - x_start
        y_grid, x_grid = np.ogrid[:y_size, :x_size]
        center_y = car_pos[1] - y_start
        center_x = car_pos[0] - x_start
        car_area_mask = (x_grid - center_x)**2 + (y_grid - center_y)**2 <= safe_radius**2
        
        # Add car area to valid mask
        valid_mask[y_start:y_end, x_start:x_end] |= car_area_mask
        
        # Set base costs for valid areas
        cost_map[valid_mask] = 1.0 + (slope_map[valid_mask] / self.config.max_slope_degrees)
        
        # Add distance-to-obstacle costs with reduced penalty
        dist_transform = cv2.distanceTransform((~obstacle_map).astype(np.uint8), cv2.DIST_L2, 5)
        safety_factor = np.clip(dist_transform / (self.config.obstacle_margin_cells), 0, 1)
        cost_map[valid_mask] *= (1.5 - 0.5 * safety_factor[valid_mask])
        
        # Extract the car area region for confidence scaling
        car_region_mask = np.zeros_like(valid_mask)
        car_region_mask[y_start:y_end, x_start:x_end] = car_area_mask
        
        # Scale by confidence except in car area
        confidence_mask = valid_mask & ~car_region_mask
        cost_map[confidence_mask] *= (1.5 - 0.5 * confidence_normalized[confidence_mask])
        
        # Debug output
        print(f"Cost at car position: {cost_map[car_pos[1], car_pos[0]]}")
        print(f"Valid cells in 5x5 area around car:")
        y_min = max(0, car_pos[1] - 2)
        y_max = min(cost_map.shape[0], car_pos[1] + 3)
        x_min = max(0, car_pos[0] - 2)
        x_max = min(cost_map.shape[1], car_pos[0] + 3)
        debug_area = cost_map[y_min:y_max, x_min:x_max]
        print(debug_area)
        
        # Additional debug info
        print(f"\nTotal valid cells: {np.sum(~np.isinf(cost_map))}")
        print(f"Valid cells in car area: {np.sum(~np.isinf(cost_map[y_start:y_end, x_start:x_end]))}")
        
        return cost_map
        
    def is_path_valid(self, start: RRTNode, end: RRTNode, cost_map: np.ndarray) -> bool:
        """Check if path between two points is traversable"""
        points = self._get_line_points(start, end)
        
        try:
            # Get costs for all points along the path
            costs = [cost_map[p[1], p[0]] for p in points]
            
            # Path is only valid if ALL points have finite cost
            valid = all(not np.isinf(c) for c in costs)
            
            if not valid:
                inf_count = sum(np.isinf(c) for c in costs)
                # print(f"Path from {(start.x, start.y)} to {(end.x, end.y)} invalid:")
                # print(f"{inf_count} of {len(costs)} points have infinite cost")
            
            return valid
            
        except IndexError as e:
            print(f"Index error checking path: {e}")
            return False
    
    def _get_line_points(self, start: RRTNode, end: RRTNode) -> List[Tuple[int, int]]:
        """Get points along line using Bresenham's algorithm"""
        points = []
        x0, y0 = start.x, start.y
        x1, y1 = end.x, end.y
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x1, y1))
        return points
    
    def find_nearest_node(self, point: RRTNode) -> RRTNode:
        """Find nearest existing node"""
        min_dist = float('inf')
        nearest = None
        
        for node in self.nodes:
            dist = np.sqrt((node.x - point.x)**2 + (node.y - point.y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = node
                
        return nearest
    
    def steer(self, from_node: RRTNode, to_point: RRTNode) -> RRTNode:
        """Create new node in direction of point"""
        dx = to_point.x - from_node.x
        dy = to_point.y - from_node.y
        dist = np.sqrt(dx*dx + dy*dy)
        
        if dist <= self.step_size:
            return RRTNode(to_point.x, to_point.y)
            
        theta = np.arctan2(dy, dx)
        new_x = int(from_node.x + self.step_size * np.cos(theta))
        new_y = int(from_node.y + self.step_size * np.sin(theta))
        
        return RRTNode(new_x, new_y)
    
    def _extract_path(self, goal_node: RRTNode) -> List[Tuple[int, int]]:
        """Extract path from goal to start"""
        path = []
        current = goal_node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return list(reversed(path))
    
    def create_path_overlay(self, path: List[Tuple[int, int]], shape: Tuple[int, int]) -> np.ndarray:
        """Create visualization of the path with debugging"""
        print(f"Creating path overlay with {len(self.nodes)} nodes")
        if path:
            print(f"Path has {len(path)} points")
        
        overlay = np.zeros((*shape, 4), dtype=np.uint8)
        
        # Draw exploration tree
        node_count = 0
        for node in self.nodes:
            if node.parent:
                cv2.line(overlay, 
                        (node.parent.x, node.parent.y),
                        (node.x, node.y),
                        (0, 255, 0, 128),  # Light green, more visible
                        1)
                node_count += 1
        print(f"Drew {node_count} exploration tree segments")
        
        # Draw final path
        if path:
            for i in range(len(path) - 1):
                # Path line
                cv2.line(overlay,
                        path[i],
                        path[i + 1],
                        (0, 0, 255, 255),  # Red, fully opaque
                        2)
                
                # Node marker
                cv2.circle(overlay, path[i], 4, (0, 255, 255, 255), -1)  # Yellow
            
            # Last node
            cv2.circle(overlay, path[-1], 4, (0, 255, 255, 255), -1)
            
            print("Drew path and nodes")
        
        return overlay

    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int], 
                cost_map: np.ndarray) -> Optional[List[Tuple[int, int]]]:
        """Generate RRT path with improved sampling"""
        print(f"\nPlanning path from {start} to {goal}")
        
        # Initialize RRT
        self.nodes = []
        start_node = RRTNode(start[0], start[1])
        self.nodes.append(start_node)
        goal_node = RRTNode(goal[0], goal[1])
        
        # Increase step size for faster exploration
        self.step_size = 30  # Increased from 20
        
        # Create a distance-weighted sampling mask
        y_grid, x_grid = np.meshgrid(np.arange(cost_map.shape[0]), np.arange(cost_map.shape[1]))
        start_distances = np.sqrt((x_grid - start[0])**2 + (y_grid - start[1])**2)
        goal_distances = np.sqrt((x_grid - goal[0])**2 + (y_grid - goal[1])**2)
        sampling_weights = 1.0 / (1.0 + 0.5 * start_distances + 0.5 * goal_distances)
        sampling_weights[cost_map == np.inf] = 0
        sampling_weights /= np.sum(sampling_weights)
        
        for i in range(self.max_iterations):
            if i % 100 == 0:
                print(f"RRT iteration {i}, nodes: {len(self.nodes)}")
            
            # Sample point with bias
            if np.random.random() < self.goal_sample_rate:
                point = goal_node
            else:
                # Sample based on distance-weighted distribution
                flat_indices = np.random.choice(
                    sampling_weights.size, 
                    p=sampling_weights.flatten()
                )
                y, x = np.unravel_index(flat_indices, sampling_weights.shape)
                point = RRTNode(x, y)
            
            # Extend tree
            nearest = self.find_nearest_node(point)
            if nearest is None:
                continue
                
            new_node = self.steer(nearest, point)
            if self.is_path_valid(nearest, new_node, cost_map):
                new_node.parent = nearest
                self.nodes.append(new_node)
                
                # Try to connect to goal periodically
                if len(self.nodes) % 10 == 0:
                    if self.is_path_valid(new_node, goal_node, cost_map):
                        print(f"Found path to goal after {i} iterations!")
                        goal_node.parent = new_node
                        self.nodes.append(goal_node)
                        return self._extract_path(goal_node)
        
        print("Failed to find path after maximum iterations")
        return None

class AdvancedNavigationSystem:
    """Main navigation system using terrain analysis and advanced path planning"""
    def __init__(self):
        self.config = NavigationConfig()
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.terrain_analyzer = TerrainAnalyzer(self.config)
        self.path_planner = PathPlanner(self.config)
            
    def get_target_coords(self):
        """Get target coordinates from Redis and convert to map coordinates"""
        target_data = self.redis_client.get('target_car_coords')
        if target_data:
            coords = struct.unpack('%sf' % 3, target_data)
            # Convert from meters to map pixels, adjusted for new resolution
            map_x = int(coords[0] * 50 + self.config.map_width / 2)  # Was *100
            map_y = int(self.config.map_height - coords[2] * 50 - 75)  # Was 150 (car position)
            return (map_x, map_y)
        return None
    
    def process_navigation_cycle(self):
        """Process one complete navigation cycle with debugging"""
        print("\n=== Starting Navigation Cycle ===")
        
        # Record that navigation is running (1 second timeout)
        self.redis_client.psetex('log_navigation_running', 1000, 'on')
        
        try:
            # Get target position from Redis
            target_coords = self.get_target_coords()
            if target_coords is None:
                print("No target coordinates found")
                return
            print(f"Target coordinates: {target_coords}")

            # Get map data and analyze terrain 
            print("Analyzing terrain...")
            self.terrain_analyzer.analyze_terrain()
            maps = self.terrain_analyzer.get_map_data()
            
            # Verify map dimensions
            expected_shape = (self.config.map_height, self.config.map_width)
            if any(m.shape != expected_shape for m in maps.values()):
                print("Error: Mismatched map dimensions")
                return
            
            # Get slope and obstacle maps
            print("Computing slope and obstacle maps...")
            slope_map = self.terrain_analyzer.compute_slope_map(maps['map'], maps['map_confidence'])
            obstacle_map = self.terrain_analyzer.identify_obstacles(maps['map'], slope_map, maps['map_confidence'])
            
            print("Creating cost map...")
            cost_map = self.path_planner.create_cost_map(
                slope_map=slope_map,
                obstacle_map=obstacle_map,
                confidence_map=maps['map_confidence']
            )
            
            # Plan path from car position to target
            start = (100, 125)  # Car position (adjusted for 400x400)
            print(f"\nPlanning path from {start} to {target_coords}")
            path = self.path_planner.plan_path(start, target_coords, cost_map)
            
            if path:
                print(f"Path found with {len(path)} points")
                path_overlay = self.path_planner.create_path_overlay(path, cost_map.shape)
                print("Created path overlay")
                self.terrain_analyzer._send_overlay_to_redis('overlay_path', path_overlay)
            else:
                print("No path found!")
            
        except Exception as e:
            print(f"Error in navigation cycle: {e}")
            
        print("=== Navigation Cycle Complete ===\n")

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
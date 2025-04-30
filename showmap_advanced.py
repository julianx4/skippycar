#showmap_advanced.py
import numpy as np
import redis
import struct
import cv2
import time
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple
from config import MapConfig

@dataclass
class ShowMapConfig:
    """Configuration for visualization parameters that don't depend on resolution"""
    map_refresh: float = 0.2
    font = cv2.FONT_HERSHEY_SIMPLEX
    camera_fov: float = 90.0
    base_height: int = 100

class MapLayer:
    """Represents a single visualization layer"""
    def __init__(self, name: str, color: Tuple[int, int, int], alpha: float = 1.0):
        self.name = name
        self.color = color
        self.alpha = alpha
        self.data: Optional[np.ndarray] = None
        self.enabled = True

class MapVisualizer:
    def __init__(self):
        # Get map dimensions from MapConfig
        map_config = MapConfig()
        
        # Get visualization parameters from ShowMapConfig
        show_config = ShowMapConfig()
        
        # Combine them
        self.config = show_config
        self.config.width = map_config.width
        self.config.height = map_config.height
        self.config.cm_per_pixel = map_config.get_resolution()
        self.config.car_position_on_map = 250 // self.config.cm_per_pixel
        
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
        # Updated layers to include force vectors
        self.layers = {
            'base': MapLayer('raw_map', (128, 128, 128)),      # Base map in gray
            'path': MapLayer('overlay_path', (255, 0, 0), 1.0), # Path overlay (keep for compatibility)
            'forces': MapLayer('overlay_forces', (0, 0, 255), 0.8)  # Force vectors
        }
        
        # Add configuration for confidence threshold
        self.confidence_threshold = 5  # Show cells with confidence > 8/15

        # Status colors
        self.car_color = (0, 100, 255)  # Orange for car
        self.target_line_color = (0, 0, 255)  # Red for target line
        self.text_color = (255, 255, 255)  # White for text

        # Add colors for force vectors
        self.force_colors = {
            'attract': (0, 255, 0),    # Green for attractive force
            'repel': (0, 0, 255),      # Red for repulsive force
            'total': (255, 255, 0)     # Yellow for total force
        }

        # Add navigation colors to existing colors
        self.nav_colors = {
            'scanned_path': (50, 50, 200),   # Light blue for scanned paths
            'best_path': (0, 255, 0),        # Green for best path
            'rejected_path': (200, 50, 50)    # Red for rejected paths
        }

    def get_redis_float(self, name: str, default: float = 0) -> float:
        """Helper to get and convert Redis values"""
        value = self.redis_client.get(name)
        if value is None:
            return default
        return float(value)

    def get_redis_map(self, name: str) -> Optional[np.ndarray]:
        """Enhanced map retrieval to handle force overlay"""
        encoded = self.redis_client.get(name)
        if encoded is None:
            return None
            
        h, w = struct.unpack('>II', encoded[:8])
        
        # Add navigation overlay handling
        if name == 'overlay_navigation':
            # Navigation overlay includes path scores and selection
            return np.frombuffer(encoded[8:], dtype=np.uint8).reshape(h, w, 4)
            
        # Handle different map types
        if name == 'raw_map':
            data = np.frombuffer(encoded[8:], dtype=np.uint16).reshape(h, w)
            
            # Extract height and confidence
            height = ((data >> 4) & 0x0FFF).astype(np.int16)
            confidence = (data & 0x0F)
            
            # Create visualization array
            vis_map = np.full((h, w), self.config.base_height, np.uint8)
            confident_cells = confidence > self.confidence_threshold
            vis_map[confident_cells] = height[confident_cells] + self.config.base_height
            
            return vis_map
            
        elif name == 'overlay_forces':
            # Force overlay includes alpha channel
            return np.frombuffer(encoded[8:], dtype=np.uint8).reshape(h, w, 4)
            
        else:
            # Other overlays (path, etc.)
            return np.frombuffer(encoded[8:], dtype=np.uint8).reshape(h, w)
        
    def draw_car_and_cone(self, map_image: np.ndarray, scale: float) -> None:
        # Car dimensions in display pixels
        car_width_pixels = int(25 / self.config.cm_per_pixel * scale)
        car_length_pixels = int(55 / self.config.cm_per_pixel * scale)
        
        # Car position in display coordinates
        car_center_x = int(self.config.width / 2 * scale)
        car_y = int((self.config.height - self.config.car_position_on_map) * scale)
        
        # Draw car rectangle
        top_left = (
            int(car_center_x - car_width_pixels//2),
            int(car_y + car_length_pixels)
        )
        bottom_right = (
            int(car_center_x + car_width_pixels//2),
            int(car_y)
        )
        cv2.rectangle(map_image, top_left, bottom_right, self.car_color, -1)
        
        # Draw cone
        camera_y = car_y
        distance_to_top = camera_y
        fov_rad = np.radians(self.config.camera_fov)
        cone_width = int(2 * distance_to_top * np.tan(fov_rad / 2))
        
        visible_cone = np.array([
            [car_center_x, camera_y],
            [car_center_x - cone_width//2, 0],
            [car_center_x + cone_width//2, 0]
        ], np.int32)
        
        visible_cone = visible_cone.reshape((-1, 1, 2))
        cv2.polylines(map_image, [visible_cone], True, (255, 255, 255), 1)

    def draw_debug_info(self, map_image: np.ndarray) -> None:
        # Debug info drawn at fixed positions in 800x800 window
        debug_info = self._get_debug_info()
        
        # Draw left column (system status)
        count = 1
        for text, value in debug_info['left'].items():
            count += 1
            # Positions now relative to 800x800 window
            position1 = (40, 600 + 20 * count)
            position2 = (280, 600 + 20 * count)
            cv2.putText(map_image, str(text), position1,
                    self.config.font, 0.6, self.text_color, 1)
            cv2.putText(map_image, str(value), position2,
                    self.config.font, 0.6, self.text_color, 1)

        # Draw right column (measurements)
        count = 1
        for text, value in debug_info['right'].items():
            count += 1
            position1 = (374, 600 + 20 * count)
            position2 = (620, 600 + 20 * count)
            cv2.putText(map_image, str(text), position1,
                    self.config.font, 0.6, self.text_color, 1)
            cv2.putText(map_image, str(value), position2,
                    self.config.font, 0.6, self.text_color, 1)
            
    def _get_debug_info(self) -> Dict:
        """Collect all debug information from Redis"""
        # Get all the required values from Redis
        log_sensing_time = round(self.get_redis_float('log_sensing_time'), 2)
        log_target_distance = round(self.get_redis_float('log_target_distance'), 2)
        log_target_angle = round(self.get_redis_float('log_target_angle'), 2)
        log_current_speed = round(self.get_redis_float('current_speed'), 2)
        log_in_front_of_car = self.get_redis_float('log_in_front_of_car')
        log_uptime = int(self.get_redis_float('log_uptime'))

        # Get status values
        status_values = {}
        for service in ['sensing', 'navigation', 'batterymeter', 'driving']:
            value = self.redis_client.get(f'log_{service}_running')
            status_values[service] = value.decode('utf-8') if value else 'off'

        # Get camera detection status
        detect_cam = self.redis_client.get('log_detect_cam')
        detect_cam = detect_cam.decode('utf-8') if detect_cam else 'None'

        # Get battery voltages
        voltages = self.redis_client.get('voltages')
        if voltages:
            voltages = np.round(np.array(struct.unpack('%sf' % 2, voltages)), 2)
            voltages = f"{voltages[0]} {voltages[1]}"
        else:
            voltages = "0 0"

        # Get navigation values
        angle = round(self.get_redis_float('angle', 0), 2)
        target_speed = round(self.get_redis_float('target_speed', 0), 2)

        return {
            'left': {
                'sensing': status_values['sensing'],
                'navigation': status_values['navigation'],
                'batterymeter': status_values['batterymeter'],
                'driving': status_values['driving'],
                'detect cam': detect_cam
            },
            'right': {
                'battery voltages': voltages,
                'sensing time': log_sensing_time,
                'target dist, angle': f"{log_target_distance} {log_target_angle}",
                'angle': angle,
                'target speed': target_speed,
                'current speed': log_current_speed,
                'obstacle height': log_in_front_of_car,
                'uptime': log_uptime
            }
        }

    def add_debug_visualizations(self, map_image: np.ndarray) -> None:
        """Add debug visualizations to the map image"""
        # Get debug cone mask if it exists
        debug_cone = self.get_redis_map('debug_cone_mask')
        if debug_cone is not None:
            # Resize to match display size
            debug_cone = cv2.resize(debug_cone, (800, 800), interpolation=cv2.INTER_NEAREST)
            # Overlay in blue with 50% transparency
            blue_overlay = np.zeros_like(map_image)
            blue_overlay[debug_cone > 0] = [255, 0, 0]  # Blue color
            cv2.addWeighted(map_image, 0.7, blue_overlay, 0.3, 0, map_image)

    def draw_target_line(self, map_image: np.ndarray, scale: float) -> None:
        target_coords = self.redis_client.get('target_car_coords')
        if target_coords is not None:
            coords = np.array(struct.unpack('%sf' % 3, target_coords))
            
            # Scale coordinates to display size
            pixels_per_meter = 100 / self.config.cm_per_pixel * scale
            mx = int(coords[0] * pixels_per_meter + self.config.width * scale / 2)
            my = int(self.config.height * scale - self.config.car_position_on_map * scale - coords[2] * pixels_per_meter)
            
            # Start from car front
            car_y = int((self.config.height - self.config.car_position_on_map) * scale)
            start_point = (int(self.config.width * scale // 2), car_y)
            end_point = (mx, my)
            
            cv2.line(map_image, start_point, end_point, self.target_line_color, thickness=2)

    def create_visualization(self) -> np.ndarray:
        """Enhanced visualization with navigation paths"""
        try:
            # Get base map with only confident data shown
            base_map = self.get_redis_map('raw_map')
            if base_map is None or base_map.size == 0:
                # Create a blank base map if no data
                base_map = np.full((self.config.height, self.config.width), self.config.base_height, dtype=np.uint8)
                print("Warning: No map data received from Redis, using blank map")
            
            # Scale up the map to 800x800 display size
            map_image = cv2.resize(base_map, (800, 800), interpolation=cv2.INTER_NEAREST)
            map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
            
            # Add path overlay if exists
            path_data = self.get_redis_map('overlay_path')
            if path_data is not None and len(path_data.shape) == 3 and path_data.shape[2] == 4:
                path_data = cv2.resize(path_data, (800, 800), interpolation=cv2.INTER_NEAREST)
                alpha = path_data[:, :, 3:] / 255.0
                map_image = map_image * (1 - alpha) + path_data[:, :, :3] * alpha
            
            # Calculate scale factor for drawing
            scale = 800 / self.config.width
            
            # Draw with scaled coordinates
            self.draw_car_and_cone(map_image, scale)
            self.draw_target_line(map_image, scale)
            self.draw_debug_info(map_image)
            
            # Add navigation visualization if available
            nav_data = self.get_redis_map('overlay_navigation')
            if nav_data is not None and len(nav_data.shape) == 3 and nav_data.shape[2] == 4:
                nav_data = cv2.resize(nav_data, (800, 800), interpolation=cv2.INTER_NEAREST)
                alpha = nav_data[:, :, 3:] / 255.0
                map_image = map_image * (1 - alpha) + nav_data[:, :, :3] * alpha
            
            # Draw additional navigation info
            self.draw_navigation_info(map_image)
            
            return map_image.astype(np.uint8)
                
        except Exception as e:
            print(f"Error in create_visualization: {str(e)}")
            return self._create_error_image(str(e))

    def _create_error_image(self, error_text: str) -> np.ndarray:
        """Create an error image with the error message"""
        error_image = np.zeros((800, 800, 3), dtype=np.uint8)
        cv2.putText(error_image, "Error:", (50, 350),
                   self.config.font, 1.0, (0, 0, 255), 2)
        cv2.putText(error_image, error_text, (50, 400),
                   self.config.font, 0.8, (0, 0, 255), 1)
        return error_image

    def draw_navigation_info(self, map_image: np.ndarray) -> None:
        """Draw navigation-related debug information"""
        # Get navigation parameters
        angle = self.get_redis_float('angle', 0)
        target_speed = self.get_redis_float('target_speed', 0)
        look_ahead = self.get_redis_float('nav_look_ahead', 0)
        scan_width = self.get_redis_float('nav_scan_width', 0)
        
        # Draw navigation status (in top-left, above existing debug info)
        y_offset = 50  # Start above existing debug info
        cv2.putText(map_image, f"Nav Status:", 
                    (40, y_offset), self.config.font, 0.6, (255, 255, 255), 1)
        cv2.putText(map_image, f"Steering: {angle:.1f}°", 
                    (40, y_offset + 20), self.config.font, 0.6, (255, 255, 255), 1)
        cv2.putText(map_image, f"Target Speed: {target_speed:.1f}", 
                    (40, y_offset + 40), self.config.font, 0.6, (255, 255, 255), 1)
        
        # Draw scan parameters if available
        if look_ahead > 0 and scan_width > 0:
            cv2.putText(map_image, f"Look Ahead: {look_ahead:.0f}cm", 
                        (40, y_offset + 60), self.config.font, 0.6, (255, 255, 255), 1)
            cv2.putText(map_image, f"Scan Width: {scan_width:.0f}cm", 
                        (40, y_offset + 80), self.config.font, 0.6, (255, 255, 255), 1)

    def run(self):
        """Main visualization loop"""
        while True:
            try:
                map_image = self.create_visualization()
                
                cv2.namedWindow('map', cv2.WINDOW_NORMAL)
                cv2.imshow('map', map_image)
                
                time.sleep(self.config.map_refresh)
                
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
                    
            except Exception as e:
                print(f"Error in visualization loop: {e}")
                time.sleep(1)


if __name__ == "__main__":
    visualizer = MapVisualizer()
    visualizer.run()
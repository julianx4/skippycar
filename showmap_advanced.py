#showmap_advanced.py
import numpy as np
import redis
import struct
import cv2
import time
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple

@dataclass
class MapConfig:
    """Configuration for map visualization"""
    width: int = 400    
    height: int = 400   
    base_height: int = 100
    map_refresh: float = 0.2
    font = cv2.FONT_HERSHEY_SIMPLEX
    cm_per_pixel: int = 2  
    camera_fov: float = 90.0  # Camera field of view in degrees
    car_position_on_map: int = 250 // cm_per_pixel  # Same as sensing_advanced.py
    


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
        self.config = MapConfig()
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
        # Define visualization layers with their colors and blend modes
        self.layers = {
            'base': MapLayer('raw_height_map', (128, 128, 128)),  # Base height map in gray
            'path': MapLayer('overlay_path', (255, 0, 0), 1.0),  # Path overlay with full opacity
            'obstacle': MapLayer('overlay_obstacles', (255, 0, 0), 0.7),  # Obstacles in red
            'slopes': MapLayer('overlay_slopes', (0, 0, 255), 0.5),  # Slopes in blue
            'confidence': MapLayer('raw_confidence_map', (0, 255, 0), 0.3),  # Confidence in green
        }

        # Status colors
        self.car_color = (0, 100, 255)  # Orange for car
        self.target_line_color = (0, 0, 255)  # Red for target line
        self.text_color = (255, 255, 255)  # White for text

    def get_redis_float(self, name: str, default: float = 0) -> float:
        """Helper to get and convert Redis values"""
        value = self.redis_client.get(name)
        if value is None:
            return default
        return float(value)

    def get_redis_map(self, name: str) -> Optional[np.ndarray]:
        """Retrieve map data from Redis with proper type handling"""
        encoded = self.redis_client.get(name)
        if encoded is None:
            return np.full((self.config.height, self.config.width), 
                          self.config.base_height, np.uint8)
            
        h, w = struct.unpack('>II', encoded[:8])
        data = encoded[8:]
        
        # Handle RGBA data
        if len(data) == h * w * 4:
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 4)
        else:
            # Handle single-channel data
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w)
        
    def draw_car_and_cone(self, map_image: np.ndarray) -> None:
        """Draw car rectangle and visible cone using explicit angle from camera position"""
        # Car dimensions
        car_width_pixels = 13    # 25cm / 2cm per pixel
        car_length_pixels = 28   # 55cm / 2cm per pixel
        
        # Car position - FIXED to match terrain mapping
        car_center_x = self.config.width // 2
        car_y = self.config.height - self.config.car_position_on_map
        
        # Draw car rectangle in orange
        cv2.rectangle(map_image, 
                    (car_center_x - car_width_pixels//2, car_y + car_length_pixels),
                    (car_center_x + car_width_pixels//2, car_y),                  
                    self.car_color, -1)
        
        # Calculate cone from front of car 
        camera_y = car_y #+ car_length_pixels  # Camera at front of car
        
        # Calculate cone points based on FOV angle
        fov_rad = np.radians(self.config.camera_fov)
        distance_to_top = camera_y
        
        # Calculate cone width at map top using trigonometry
        cone_width = int(2 * distance_to_top * np.tan(fov_rad / 2))
        
        # Define cone vertices starting from camera position
        visible_cone = np.array([
            [car_center_x, camera_y],  # Camera position (cone apex)
            [car_center_x - cone_width//2, 0],  # Left edge at top of map
            [car_center_x + cone_width//2, 0]   # Right edge at top of map
        ], np.int32)
        
        visible_cone = visible_cone.reshape((-1, 1, 2))
        cv2.polylines(map_image, [visible_cone], True, (255, 255, 255), 1)
        
    def draw_debug_info(self, map_image: np.ndarray) -> None:
        """Draw debug information with adjusted positions"""
        debug_info = self._get_debug_info()
        
        # Draw left column (system status)
        count = 1
        for text, value in debug_info['left'].items():
            count += 1
            cv2.putText(map_image, str(text), (20, 300 + 10 * count),  # Was (20, 300...)
                    self.config.font, 0.3, self.text_color, 1)       # Was 0.4
            cv2.putText(map_image, str(value), (140, 300 + 10 * count), # Was (140, 300...)
                    self.config.font, 0.3, self.text_color, 1)

        # Draw right column (measurements)
        count = 1
        for text, value in debug_info['right'].items():
            count += 1
            cv2.putText(map_image, str(text), (187, 300 + 10 * count),  # Was (187, 300...)
                    self.config.font, 0.3, self.text_color, 1)       # Was 0.4
            cv2.putText(map_image, str(value), (310, 300 + 10 * count), # Was (310, 300...)
                    self.config.font, 0.3, self.text_color, 1)

    def _get_debug_info(self) -> Dict:
        """Collect all debug information from Redis"""
        # Get all the required values from Redis
        log_sensing_time = round(self.get_redis_float('log_sensing_time'), 2)
        log_target_distance = round(self.get_redis_float('log_target_distance'), 2)
        log_target_angle = round(self.get_redis_float('log_target_angle'), 2)
        log_path = self.get_redis_float('path')
        log_path_min_cost = round(self.get_redis_float('path_min_cost'), 2)
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
                'current path': log_path,
                'path min cost': log_path_min_cost,
                'current speed': log_current_speed,
                'obstacle height': log_in_front_of_car,
                'uptime': log_uptime
            }
        }


    def draw_target_line(self, map_image: np.ndarray) -> None:
        """Draw line to target with scaled coordinates"""
        target_coords = self.redis_client.get('target_car_coords')
        if target_coords is not None:
            coords = np.array(struct.unpack('%sf' % 3, target_coords))
            
            # Scale coordinates: 50 pixels per meter (since we're at 2cm/pixel)
            mx = int(coords[0] * 50 + self.config.width / 2)
            my = int(self.config.height - self.config.car_position_on_map - coords[2] * 50)
            
            # Start from car front
            car_y = self.config.height - self.config.car_position_on_map
            
            cv2.line(map_image, 
                    (self.config.width//2, car_y),  # Start from car front center
                    (mx, my),
                    self.target_line_color, thickness=2)

    def create_visualization(self) -> np.ndarray:
        """Create complete map visualization with proper layer blending"""
        try:
            # Start with base map
            base_map = self.get_redis_map('raw_height_map')
            map_image = cv2.cvtColor(base_map, cv2.COLOR_GRAY2BGR)
            
            # # Process each layer
            # for layer_name, layer in self.layers.items():
            #     if not layer.enabled or layer_name == 'base':
            #         continue
                
            #     layer_data = self.get_redis_map(layer.name)
            #     if layer_data is None:
            #         continue

            #     if layer_name == 'path' and len(layer_data.shape) == 3 and layer_data.shape[2] == 4:
            #         pass # really nothing?
            #     elif len(layer_data.shape) == 3 and layer_data.shape[2] == 4:
            #         pass # really nothing?
            #     else:
            #         normalized = cv2.normalize(layer_data, None, 0, 1, cv2.NORM_MINMAX)
            #         mask = normalized > 0.1
            #         if layer_name == 'slopes':
            #             colored = cv2.applyColorMap((normalized * 255).astype(np.uint8), 
            #                                     cv2.COLORMAP_JET)
            #             map_image = cv2.addWeighted(map_image, 1.0, colored, layer.alpha, 0)
            #         else:
            #             overlay = np.zeros_like(map_image)
            #             overlay[mask] = np.array(layer.color)
            #             map_image = cv2.addWeighted(map_image, 1.0, overlay, layer.alpha, 0)

            # map_image = map_image.astype(np.uint8)
            
            self.draw_car_and_cone(map_image)
            self.draw_target_line(map_image)
            self.draw_debug_info(map_image)
            
            return map_image

        except Exception as e:
            print(f"Error in create_visualization: {str(e)}")
            import traceback
            traceback.print_exc()
            return None

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
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
        """Draw car rectangle and visible cone"""
        # Draw car rectangle in orange
        cv2.rectangle(map_image, (187, 242), (213, 305), self.car_color, -1)
        
        # Draw visible cone in white
        visible_cone = np.array([[213, 242], [187, 242], [0, 0], [400, 0]], np.int32)
        visible_cone = visible_cone.reshape((-1, 1, 2))
        cv2.polylines(map_image, [visible_cone], True, (255, 255, 255), 1)

    def draw_debug_info(self, map_image: np.ndarray) -> None:
        """Draw all debug information"""
        # Get all the debug values
        debug_info = self._get_debug_info()
        
        # Draw left column (system status)
        count = 1
        for text, value in debug_info['left'].items():
            count += 1
            cv2.putText(map_image, str(text), (20, 300 + 10 * count), 
                       self.config.font, 0.4, self.text_color, 1)
            cv2.putText(map_image, str(value), (140, 300 + 10 * count), 
                       self.config.font, 0.4, self.text_color, 1)

        # Draw right column (measurements)
        count = 1
        for text, value in debug_info['right'].items():
            count += 1
            cv2.putText(map_image, str(text), (187, 300 + 10 * count), 
                       self.config.font, 0.4, self.text_color, 1)
            cv2.putText(map_image, str(value), (310, 300 + 10 * count), 
                       self.config.font, 0.4, self.text_color, 1)

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
        """Draw line to target if target exists"""
        target_coords = self.redis_client.get('target_car_coords')
        if target_coords is not None:
            coords = np.array(struct.unpack('%sf' % 3, target_coords))
            mx = int(coords[0] * 100 + self.config.width / 2)
            my = int(self.config.height - coords[2] * 100)
            cv2.line(map_image, 
                    (int(self.config.width/2), self.config.height - 150),
                    (mx, my - 150), 
                    self.target_line_color, thickness=3)

    def create_visualization(self) -> np.ndarray:
        """Create complete map visualization with proper layer blending"""
        # Start with base map
        base_map = self.get_redis_map('raw_height_map')
        map_image = cv2.cvtColor(base_map, cv2.COLOR_GRAY2BGR)
        
        # Process each layer
        for layer_name, layer in self.layers.items():
            if not layer.enabled or layer_name == 'base':
                continue
            
            layer_data = self.get_redis_map(layer.name)
            if layer_data is None:
                continue

            # Special handling for path overlay
            if layer_name == 'path' and len(layer_data.shape) == 3 and layer_data.shape[2] == 4:
                # Extract RGB and alpha channels
                rgb = layer_data[..., :3].astype(float)
                alpha = layer_data[..., 3].astype(float) / 255.0
                
                # Create alpha mask for each channel
                alpha_3d = np.stack([alpha] * 3, axis=-1)
                
                # Blend the path overlay
                mask = alpha > 0
                map_image[mask] = (map_image[mask] * (1 - alpha_3d[mask]) + 
                                rgb[mask] * alpha_3d[mask]).astype(np.uint8)
            elif len(layer_data.shape) == 3 and layer_data.shape[2] == 4:
                # Handle other RGBA overlays
                overlay = layer_data[..., :3].astype(float)
                alpha = layer_data[..., 3].astype(float) / 255.0 * layer.alpha
                alpha_3d = np.stack([alpha] * 3, axis=-1)
                map_image = (map_image * (1 - alpha_3d) + overlay * alpha_3d).astype(np.uint8)
            else:
                # Handle single-channel data
                normalized = cv2.normalize(layer_data, None, 0, 1, cv2.NORM_MINMAX)
                if layer_name == 'slopes':
                    colored = cv2.applyColorMap((normalized * 255).astype(np.uint8), 
                                            cv2.COLORMAP_JET)
                    map_image = cv2.addWeighted(map_image, 1.0, colored, layer.alpha, 0)
                else:
                    mask = normalized > 0.1
                    overlay = np.zeros_like(map_image)
                    overlay[mask] = np.array(layer.color)
                    map_image = cv2.addWeighted(map_image, 1.0, overlay, layer.alpha, 0)

        map_image = map_image.astype(np.uint8)
        
        # Draw additional elements
        self.draw_car_and_cone(map_image)
        self.draw_target_line(map_image)
        self.draw_debug_info(map_image)
        
        return map_image

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
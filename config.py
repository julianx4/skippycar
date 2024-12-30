# config.py
import redis

class MapConfig:
    """Central configuration for map parameters"""
    PHYSICAL_WIDTH = 800   # 8 meters
    PHYSICAL_HEIGHT = 800  # 8 meters
    BASE_HEIGHT = 100     # Base height for visualization
    
    
    def __init__(self):
        self.base_height = self.BASE_HEIGHT
        self.width = int(self.PHYSICAL_WIDTH // self.get_resolution())
        self.height = int(self.PHYSICAL_HEIGHT // self.get_resolution())
        
    
    def update_dimensions(self):
        """Calculate map dimensions based on current resolution"""
        self.cm_per_pixel = 5#self.get_resolution()
        self.width = int(self.PHYSICAL_WIDTH // self.cm_per_pixel)
        self.height = int(self.PHYSICAL_HEIGHT // self.cm_per_pixel)
        self.car_position_on_map = 250 // self.cm_per_pixel
    
    def get_resolution(self):
        """Get resolution from Redis, defaulting to 2cm if not set"""
        try:
            r = redis.Redis(host='localhost', port=6379, db=0)
            res = r.get('map_resolution')
            return float(res) if res else 2.0
        except:
            return 2.0

    def get_dimensions(self):
        """Return current dimensions as tuple for compatibility"""
        self.update_dimensions()
        return self.width, self.height, self.cm_per_pixel
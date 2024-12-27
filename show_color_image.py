import cv2
import numpy as np
import redis
import struct
import time
import os

# Fix for Qt platform plugin error
os.environ["QT_QPA_PLATFORM"] = "xcb"

# Configuration for Redis connection
redis_host = 'localhost'
redis_port = 6379
redis_db = 0

# Keys for the stored images
depth_key = 'D435_depth_image'
rgb_key = 'D435_image'

def load_image_from_redis(r, key):
    """
    Load image data from Redis
    Returns None if image is not found
    """
    encoded_image = r.get(key)
    
    if not encoded_image:
        print(f"Image data not found in Redis for key: {key}")
        return None
    
    try:
        # Decode the image data
        h, w = struct.unpack('>II', encoded_image[:8])
        
        # For depth image (2D)
        if key == depth_key:
            # Load as uint16 for depth data
            image = np.frombuffer(encoded_image, dtype=np.uint16, offset=8).reshape((h, w))
            
            # Normalize depth values for visualization
            # Scale to range 0-255 for display
            depth_scale = 255.0 / (np.maximum(np.max(image), 1))
            image = (image * depth_scale).astype(np.uint8)
            
        # For RGB image (3D)
        else:
            image = np.frombuffer(encoded_image, dtype=np.uint8, offset=8).reshape((h, w, 3))
        
        return image
        
    except ValueError as e:
        print(f"Error loading image from Redis: {e}")
        print(f"Image size: {len(encoded_image)}, Expected shape: ({h}, {w})")
        return None
    except Exception as e:
        print(f"Unexpected error loading image: {e}")
        return None

def display_images():
    # Connect to Redis
    r = redis.Redis(host=redis_host, port=redis_port, db=redis_db)
    
    try:
        while True:
            # Load both RGB and depth images
            rgb_image = load_image_from_redis(r, rgb_key)
            depth_image = load_image_from_redis(r, depth_key)
            
            # Display images if available
            if rgb_image is not None:
                cv2.imshow('RGB Image', rgb_image)
            
            if depth_image is not None:
                # Apply colormap to depth image for better visualization
                depth_colormap = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
                cv2.imshow('Depth Image', depth_colormap)
            
            # Break loop if 'q' is pressed
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            
    except KeyboardInterrupt:
        print("Stopping visualization...")
    except Exception as e:
        print(f"Error in display loop: {e}")
    
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        r.close()

if __name__ == '__main__':
    try:
        display_images()
    except Exception as e:
        print(f"Fatal error: {e}")
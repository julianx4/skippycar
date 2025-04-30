#!/usr/bin/env python3
import redis
import numpy as np
import apriltag
import cv2
import time
import struct
import traceback
from threading import Thread

class AprilTagDetectorService:
    def __init__(self):
        print("Initializing AprilTag Detector Service...")
        try:
            self.redis = redis.Redis(host='localhost', port=6379, db=0)
            print("Redis connection established")
            
            detector_config = apriltag.DetectorOptions(
                families='tag36h11',
                border=1,
                nthreads=4,
                quad_decimate=1.0,
                quad_blur=0.0,
                refine_edges=True,
                refine_decode=True,
                refine_pose=True,
                debug=False,
                quad_contours=True
            )
            self.detector = apriltag.Detector(detector_config)
            print("AprilTag detector initialized")
            
        except Exception as e:
            print(f"Error during initialization: {e}")
            print(traceback.format_exc())
            raise

    def get_image_from_redis(self, key):
        try:
            encoded = self.redis.get(key)
            if encoded is None:
                return None
                
            # Decode image dimensions
            h, w = struct.unpack('>II', encoded[:8])
            # Convert the rest to numpy array
            img = np.frombuffer(encoded[8:], dtype=np.uint8).reshape(h, w, -1)
            return img
            
        except struct.error as e:
            print(f"Error unpacking image dimensions for key {key}: {e}")
            return None
        except Exception as e:
            print(f"Error getting image from Redis for key {key}: {e}")
            return None

    def run(self):
        print("Starting AprilTag detection loop...")
        last_error_time = 0
        error_count = 0
        
        while True:
            try:
                # Get images from Redis
                color_image = self.get_image_from_redis('D435_image')
                t265_image = self.get_image_from_redis('T265_image')
                
                if color_image is not None:
                    
                    # Process D435 image
                    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                    tags = self.detector.detect(gray_image)
                    if tags:
                        # Process first detected tag
                        tag = tags[0]
                        self.redis.set('apriltag_d435', struct.pack('2f', *tag.center))
                        self.redis.set('apriltag_source', 'D435')
                        self.redis.pexpire('apriltag_d435', 500)  # 500ms timeout
                        # Verify the data was set
                        verification = self.redis.get('apriltag_d435')
                        if verification:
                            x, y = struct.unpack('2f', verification)
                
                if t265_image is not None:       
                    # Squeeze out single-dimensional entries
                    t265_gray = np.squeeze(t265_image)
                    
                    tags = self.detector.detect(t265_gray)
                    if tags and not self.redis.exists('apriltag_d435'):
                        # Only use T265 if D435 didn't detect anything
                        tag = tags[0]
                        self.redis.set('apriltag_t265', struct.pack('2f', *tag.center))
                        self.redis.set('apriltag_source', 'T265')
                        self.redis.pexpire('apriltag_t265', 500)  # 500ms timeout
                
                # Reset error count on successful iteration
                error_count = 0
                time.sleep(0.05)  # Run at 20Hz
                
            except Exception as e:
                current_time = time.time()
                error_count += 1
                
                # Only print error if it's been more than 5 seconds since last error
                if current_time - last_error_time > 5:
                    print(f"Error in AprilTag detection (count: {error_count}): {e}")
                    print(traceback.format_exc())
                    last_error_time = current_time
                
                # If we've had too many errors, wait longer
                if error_count > 10:
                    print("Too many errors, waiting longer...")
                    time.sleep(5)
                else:
                    time.sleep(1)

if __name__ == "__main__":
    try:
        detector = AprilTagDetectorService()
        detector.run()
    except KeyboardInterrupt:
        print("\nShutting down AprilTag detector...")
    except Exception as e:
        print(f"Fatal error: {e}")
        print(traceback.format_exc()) 
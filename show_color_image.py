import cv2
import numpy as np
import redis
import struct
import time

# Configuration for Redis connection
redis_host = 'localhost'
redis_port = 6379
redis_db = 0

# Key name for the stored image
image_key = 'D435_image'

def load_image_from_redis():
    # Connect to Redis
    r = redis.Redis(host=redis_host, port=redis_port, db=redis_db)

    # Retrieve the image data from Redis
    encoded_image = r.get(image_key)

    if not encoded_image:
        print(f"Image data not found in Redis for key: {image_key}")
        return None

    # Decode the image data
    # Assuming the image was stored as width, height followed by the image bytes in BGR format
    h, w = struct.unpack('>II', encoded_image[:8])
    image = np.frombuffer(encoded_image, dtype=np.uint8, offset=8).reshape((h, w, 3))

    return image

def display_image(image):
    # Display the image using OpenCV
    cv2.imshow('Image from Redis', image)
    # Wait for 30 ms before loading the next image
    cv2.waitKey(30)

if __name__ == '__main__':
    while True:
        image = load_image_from_redis()
        if image is not None:
            display_image(image)
        else:
            # If no image is received, wait for a short while before trying again
            time.sleep(0.5)

    # Cleanup (press 'q' to quit)
    cv2.destroyAllWindows()

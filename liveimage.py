import numpy as np
import redis
import struct
import time
import cv2
from flask import Response, Flask
import pyrealsense2 as rs


r = redis.Redis(host='localhost', port=6379, db=0)

def redis_to_map(redis,name):
    encoded = redis.get(name)
    if encoded is None:
        return np.full((640,360,3),100, np.uint8)
    else:
        h, w = struct.unpack('>II', encoded[:8])
        array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 3)
        return array


app = Flask(__name__)

def gen_frames():  
    while True:
        frame = redis_to_map(r, "D435_image")

        _, buffer = cv2.imencode('.jpg', frame,[int(cv2.IMWRITE_JPEG_QUALITY), 50])
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
        time.sleep(0.1) 
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
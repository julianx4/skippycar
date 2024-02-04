import io
from flask import Flask, render_template, jsonify, request, Response
import numpy as np
import struct
import cv2
import redis
import time

app = Flask(__name__)
r = redis.Redis()

def rget_and_float(name, default = None):
    output = r.get(name)
    if output == None:
        return default
    else:
        return float(output)
    
def redis_to_map(redis,name):
    encoded = redis.get(name)
    if encoded is None:
        return np.full((640,360,3),100, np.uint8)
    else:
        h, w = struct.unpack('>II', encoded[:8])
        array = np.frombuffer(encoded, dtype=np.uint8, offset=8).reshape(h, w, 3)
        return array

tap_target_memory_time = int(rget_and_float('tap_target_memory_time', 1000))

variables = []
log_values_dict = {}
log_titles = [
    "log_uptime",
    "log_sensing_time",
    "log_target_distance",
    "log_target_angle",
    "path",
    "path_min_cost",
    "current_speed",
    "log_in_front_of_car",
    "voltage_cell1",
    "voltage_cell2",
    "log_sensing_running",
    "log_navigation_running",
    "log_batterymeter_running",
    "log_driving_running",
    "log_detect_cam"
]

@app.route("/")
def index():
    return render_template("index.jinja")

@app.route("/variables")
def api_variables():
    return jsonify(variables)

@app.route("/value", methods=['GET', 'POST'])
def api_value():
    if request.method == 'POST':
        value = request.json['value']
        name = request.json['name']
        r.set(name, str(value))
        return ''
    else:
        name = request.values.get('name')
        value = r.get(name) or 0
        return jsonify(value)

@app.route("/timed_value", methods=['POST'])
def api_timed_value():
    value = request.json['value']
    name = request.json['name']
    time = tap_target_memory_time
    r.psetex(name, int(time), int(value))
    return ''
    
@app.route("/save", methods=['POST'])
def save():
    with io.open('variables.txt', 'w', encoding='utf8') as f:
        for name in variables:
            value = r.get(name) or b'0'
            f.write('%s = %s\n' % (
                name,
                value.decode('utf8')
            ))
    return ''


def load_and_set_from_txt():
    try:
        with io.open('variables.txt', 'r', encoding='utf8') as f:
            for line in f:
                name, value = line.strip().split('=')
                name = name.strip()
                value = float(value)
                r.set(name, value)
                variables.append(name)
        #variables.sort()
    except Exception as e:
        print(f"Error loading variables.txt: {e}")


@app.route("/log_titles", methods=['GET'])
def api_log_titles():
    return jsonify(log_titles)

@app.route("/log_values", methods=['GET'])
def api_log_values():
    log_values = r.mget(log_titles)
    c = 0
    for log in log_titles:
        # Decode bytes to string or convert to int/float if necessary
        value = log_values[c]
        if value is not None:
            # Assuming the value should be a UTF-8 string, change the decode format if it's different
            decoded_value = value.decode('utf-8')
            
            # Try converting to float or int if it's a numerical value, otherwise keep as string
            try:
                decoded_value = float(decoded_value) if '.' in decoded_value else int(decoded_value)
            except ValueError:
                pass  # Keep as string if it's not a numerical value

            log_values_dict[log] = decoded_value
        else:
            log_values_dict[log] = None  # Or some default value if None is not acceptable
        c += 1

    return jsonify(log_values_dict)

@app.route("/image")
def image():
    array = redis_to_map(r, 'D435_image')  # Use your function to get the image from Redis
    _, buffer = cv2.imencode('.jpg', array)  # Encode image as JPEG
    return Response(buffer.tobytes(), mimetype='image/jpeg')  # Serve as JPEG


load_and_set_from_txt()
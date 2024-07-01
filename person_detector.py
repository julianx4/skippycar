import redis
import numpy as np
import struct
import cv2
import time
from pycoral.adapters import common, detect
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter

labels = read_label_file("coco_labels.txt")
interpreter = make_interpreter("ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite")
interpreter.allocate_tensors()

def draw_objects(image_np, objs, labels, model_input_size):
    original_height, original_width = image_np.shape[:2]
    model_width, model_height = model_input_size
    scale_x = original_width / model_width
    scale_y = original_height / model_height

    for obj in objs:
        bbox = obj.bbox
        xmin = int(bbox.xmin * scale_x)
        ymin = int(bbox.ymin * scale_y)
        xmax = int(bbox.xmax * scale_x)
        ymax = int(bbox.ymax * scale_y)
        
        # Draw bounding box
        cv2.rectangle(image_np, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
        
        # Draw label
        label = '%s: %.2f' % (labels.get(obj.id, obj.id), obj.score)
        cv2.putText(image_np, label, (xmin, max(ymin - 10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)



def detect_image(image_np, count=5, threshold=0.8):
    input_size = common.input_size(interpreter)
    image_resized = cv2.resize(image_np, input_size)
    _, scale = common.set_resized_input(interpreter, image_resized.shape[:2], lambda size: image_resized)
    person_box = None
    for _ in range(count):
        #start = time.perf_counter()
        interpreter.invoke()
        #inference_time = time.perf_counter() - start
        objs = detect.get_objects(interpreter, threshold, scale)
        #print('%.2f ms' % (inference_time * 1000))

    if not objs:
        pass

    for obj in objs:
        if obj.id == 0:
            person_box = obj.bbox
            #print("Person detected: ", person_box)
        #print(labels.get(obj.id, obj.id), 'id:', obj.id, 'score:', obj.score, 'bbox:', obj.bbox)
    draw_objects(image_np, objs, labels, (300, 300))
    if person_box is None:
        return image_np, None
    
    xmin = person_box.xmin
    ymin = person_box.ymin
    xmax = person_box.xmax
    ymax = person_box.ymax

    center_x = (((xmin + xmax) / 2) -150) / 150 * 75

    print(center_x) 
    return image_np, center_x

class RedisImageHandler:
    def __init__(self):
        self.r = redis.Redis(host='localhost', port=6379, db=0)

    def get_image_from_redis(self, name):
        encoded = self.r.get(name)
        if encoded is None:
            return None
        h, w = struct.unpack('>II', encoded[:8])
        image_data = encoded[8:]
        # Assuming the grayscale image is stored as (height, width) without color channels
        array = np.frombuffer(image_data, dtype=np.uint8).reshape((h, w))
        # Convert grayscale to BGR by duplicating the grayscale channel 3 times
        array_bgr = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
        return array_bgr
    
    def set_data(self, name, data, expiry=None):
      if expiry:
          self.r.psetex(name, expiry, data)
      else:
          self.r.set(name, data)

rdm = RedisImageHandler()

while True:
    image_np = rdm.get_image_from_redis('T265_image')
    if image_np is not None:
        #image_np = cv2.resize(image_np, (300, 300)) 
        image_np, center_x = detect_image(image_np, count = 1, threshold=0.6)  # Process and draw directly on the numpy array
        if center_x is not None:
          rdm.set_data('target_angle', str(center_x), 4000)
          rdm.set_data('target_distance', "2", 4000)
        cv2.namedWindow('person_detector', cv2.WINDOW_NORMAL)
        cv2.imshow('person_detector', image_np)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

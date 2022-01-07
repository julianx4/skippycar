import numpy as np
import cv2
import time
import math as m
import pyrealsense2 as rs
import apriltag
import redis
import struct
from PIL import Image
from scipy.spatial.transform import Rotation as R

# world coordinate system
# up +Y
# /\    _
# |     /| forward +Z
# |    /
# |   /
# |  /
# | /
# |/______> right +X

r = redis.Redis(host='localhost', port=6379, db=0)

realsenseW = 640
realsenseH = 360

mapW=200
mapH=200

camera_height=0.22 #mounting height of the depth camera vs. ground

detector = apriltag.Detector()

pipelineT265 = rs.pipeline()
configT265 = rs.config()
configT265.enable_device('908412110993') 
configT265.enable_stream(rs.stream.pose)
pipelineT265.start(configT265)

pipelineD435 = rs.pipeline()
configD435 = rs.config()
configD435.enable_device('143322074867') 
configD435.enable_stream(rs.stream.depth, realsenseW, realsenseH, rs.format.z16, 30)
configD435.enable_stream(rs.stream.color, realsenseW, realsenseH, rs.format.bgr8, 30)

pipeline_wrapper = rs.pipeline_wrapper(pipelineD435)
pipeline_profile = configD435.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    #print(s)
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    #print("The demo requires Depth camera with Color sensor")
    exit(0)

profile = pipelineD435.start(configD435)

stream_profile = profile.get_stream(rs.stream.color)
intrinsics = stream_profile.as_video_stream_profile().get_intrinsics()

align_to = rs.stream.color
align = rs.align(align_to)

show_height_map = True #otherwise it will show real colors
aligned_depth_frame = None

def pixel_to_car_coord(x, y):
    dist = aligned_depth_frame.get_distance(x, y)
    cx,cy,cz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], dist)
    c=np.array([cx,cy,cz])
    cx,cy,cz=np.matmul(rotation,c)
    cy = camera_height - cy
    return cx, cy, cz

def car_coord_to_world_coord(x, y, z):
    c=np.array([x,y,z])
    cx,cy,cz=np.matmul(rotation_yaw,c)
    cx = cam_in_world_coord_x + cx
    cy = cam_in_world_coord_y + cy
    cz = cam_in_world_coord_z + cz
    return cx, cy, cz

while True:
    start_time=time.time()
    framesT265 = pipelineT265.wait_for_frames()
    framesD435 = pipelineD435.wait_for_frames()
    pose = framesT265.get_pose_frame()
    if pose:
        data = pose.get_pose_data()
        w = data.rotation.w
        x = -data.rotation.z
        y = data.rotation.x
        z = -data.rotation.y
        cam_in_world_coord_x = data.translation.x
        cam_in_world_coord_y = data.translation.y
        cam_in_world_coord_z = -data.translation.z

        pitch =  (-m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi) + 1.7; #1.7 degree misalignment between T265 tracking camera and D435 depth camera
        roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
        yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
        #print(pitch)
    else:
        pitch = 0
        roll = 0
        yaw = 0
        cam_in_world_coord_x = 0
        cam_in_world_coord_y = 0
        cam_in_world_coord_z = 0
    rotation_roll = R.from_rotvec(roll * np.array([0, 0, 1]), degrees=True).as_matrix()
    rotation_pitch = R.from_rotvec(pitch * np.array([1, 0, 0]), degrees=True).as_matrix()
    rotation=np.matmul(rotation_roll, rotation_pitch)

    rotation_yaw = R.from_rotvec(yaw * np.array([0, 1, 0]), degrees=True).as_matrix()
    world_coord_rotation = np.matmul(rotation, rotation_yaw)

    #wx,wy,wz = car_coord_to_world_coord(0,0,0)
    #print(round(wx,2), round(wy,2), round(wz,2))

    aligned_frames = align.process(framesD435)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not color_frame:
        continue
    color_image = np.asanyarray(color_frame.get_data())
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    map = np.zeros((mapW,mapH,3), np.uint8)
    #show_height_map = not show_height_map # switch between height map and real colors
    for x in range(0, realsenseW, 60):
        for y in range (0, realsenseH, 10):
            cx, cy, cz = pixel_to_car_coord(x,y)
            if cy > 0.4:					#ignore obstacles above car height
                continue

            ### map drawing 2x2m
            mx = int(cx * 100 + mapW / 2)
            my = int(mapH - cz * 100)
            if show_height_map:
                c = int(128 + cy * 128 * 5)
                if c < 0 or c > 255:
                    continue

                cv2.circle(map, (mx,my), 3, (c,c,c), thickness=-1, lineType=8, shift=0)
				
            else:
                r,g,b = color_image[y,x]
                cv2.circle(map, (mx,my), 3, (int(r), int(g), int(b)), thickness=-1, lineType=8, shift=0)
	
    for result in detector.detect(gray):
        x,y = result.center
        cx, cy, cz = pixel_to_car_coord(int(x),int(y))

        targetx, targety, targetz = car_coord_to_world_coord(cx, cy, cz)

        ### send target coordinates to redis server
        target_coords_bytes = struct.pack('%sf' %3,* [targetx, targety, targetz])
        r.psetex('target_coords', 1000, target_coords_bytes) #target coordinates expire after xx milliseconds
        
        ### draw target in map
        mx = int(cx * 100 + mapW / 2)
        my = int(mapH - cz * 100)	
        cv2.circle(map, (mx,my), 3, (0,0,255), thickness=-1, lineType=8, shift=0)
        cv2.line(map, (int(mapW/2), mapH), (mx, my), (0, 0, 255), thickness=3)
	
    for quadrant_line in range(0,mapW,int(mapW/10)):
        cv2.line(map, (quadrant_line, 0), (quadrant_line, mapW), (255, 0, 0), thickness=1)
        cv2.line(map, (0, quadrant_line), (mapH,quadrant_line), (255, 0, 0), thickness=1)

			
    cv2.namedWindow('map', cv2.WINDOW_NORMAL)
    cv2.imshow('map', map)
    #print(1/(time.time()-start_time), "fps")

    carx, cary, carz = car_coord_to_world_coord(0, 0, 0)
    ### send target coordinates to redis server
    car_coords_bytes = struct.pack('%sf' %3,* [carx, cary, carz])
    r.psetex('car_coords', 1000, car_coords_bytes) #car coordinates expire after xx milliseconds
    r.psetex('yaw', 1000, yaw) #yaw expire after xx milliseconds

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

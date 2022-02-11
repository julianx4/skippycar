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

mapW = 400
mapH = 400

camera_height=0.25 #mounting height of the depth camera vs. ground

detector = apriltag.Detector()

pipelineT265 = rs.pipeline()
configT265 = rs.config()
configT265.enable_device('908412110993') 
configT265.enable_stream(rs.stream.pose)
deviceT265 = configT265.resolve(pipelineT265).get_device()
pose_sensor = deviceT265.first_pose_sensor()
pose_sensor.set_option(rs.option.enable_map_preservation, 1)
pose_sensor.set_option(rs.option.enable_relocalization, 1)
pose_sensor.set_option(rs.option.enable_pose_jumping, 1)
pose_sensor.set_option(rs.option.enable_mapping, 1)


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
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    exit(0)

profile = pipelineD435.start(configD435)

stream_profile = profile.get_stream(rs.stream.color)
intrinsics = stream_profile.as_video_stream_profile().get_intrinsics()

align_to = rs.stream.color
align = rs.align(align_to)

aligned_depth_frame = None

def map_to_redis(redis,array,name):
    h, w = array.shape[:2]
    shape = struct.pack('>II',h,w)
    encoded = shape + array.tobytes()
    #redis.setex(name,3,encoded)
    redis.set(name,encoded)
    return

def rotate_and_move_map(map, angle, up, right):
    image_center = tuple(np.array(map.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    move_mat = np.float32([
	[1, 0, right],
	[0, 1, up]])
    result_rot = cv2.warpAffine(map, rot_mat, map.shape[1::-1], flags=cv2.INTER_LINEAR, borderValue=(100,100,100))
    result = cv2.warpAffine(result_rot, move_mat, (result_rot.shape[1], result_rot.shape[0]), flags=cv2.INTER_LINEAR, borderValue=(100,100,100))
    return result

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
    cx = car_in_world_coord_x + cx
    cy = car_in_world_coord_y + cy
    cz = car_in_world_coord_z + cz
    return cx, cy, cz

map = np.full((mapW,mapH,1),100, np.uint8)
yaw = 0
last_time_clear_map = time.time()
car_in_world_coord_z = 0
car_in_world_coord_x = 0
start_time=time.time()
last_time_pipe_restart=time.time()
yaw_temp = 0
car_in_world_coord_x_temp = 0
car_in_world_coord_y_temp = 0
car_in_world_coord_z_temp = 0
app_start_time=time.time()
last_time_3d_image = time.time()
try:
    while True:
        #print(time.time()-start_time)
        start_time=time.time()
        #if time.time()-start_time > 15:
        #    pipelineT265.stop()
        #    pipelineT265.start(configT265)
        #    start_time=time.time()
        #    print("restarting pipe")
        car_in_world_coord_x_previous = car_in_world_coord_x
        car_in_world_coord_z_previous = car_in_world_coord_z
        yaw_previous = yaw
        
        framesT265 = pipelineT265.wait_for_frames()
        framesD435 = pipelineD435.wait_for_frames()
        pose = framesT265.get_pose_frame()
        if pose:
            data = pose.get_pose_data()
            #print(data.rotation, data.translation)
            w = data.rotation.w
            x = -data.rotation.z
            y = data.rotation.x
            z = -data.rotation.y
            car_in_world_coord_x = data.translation.x + car_in_world_coord_x_temp
            car_in_world_coord_y = data.translation.y + car_in_world_coord_y_temp
            car_in_world_coord_z = -data.translation.z + car_in_world_coord_z_temp

            pitch =  (-m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi) + 1.7; #1.7 degree misalignment between T265 tracking camera and D435 depth camera
            roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi 
            yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi + yaw_temp
            if time.time() - last_time_pipe_restart > 30:
                car_in_world_coord_x_temp = 0 #car_in_world_coord_x
                car_in_world_coord_y_temp = 0 #car_in_world_coord_y
                car_in_world_coord_z_temp = 0 #car_in_world_coord_z
                yaw_temp = 0 #yaw
                before_restart = time.time()
                pipelineT265.stop()                 #restarting T265 because it doesn't work properly on RPi4
                deviceT265.hardware_reset()
                print("reset done")
                configT265.enable_device('908412110993') 
                configT265.enable_stream(rs.stream.pose)
                pipelineT265.start(configT265)
                last_time_pipe_restart = time.time()                
                map = np.full((mapW,mapH,1),100, np.uint8)
                print("T265 restarted in " , str(time.time()-before_restart) , " seconds")
            #print(pitch)
        else:
            pitch = 0
            roll = 0
            yaw = 0
            car_in_world_coord_x = 0
            car_in_world_coord_y = 0
            car_in_world_coord_z = 0
        
        rotation_roll = R.from_rotvec(roll * np.array([0, 0, 1]), degrees=True).as_matrix()
        rotation_pitch = R.from_rotvec(pitch * np.array([1, 0, 0]), degrees=True).as_matrix()
        rotation=np.matmul(rotation_roll, rotation_pitch)

        rotation_yaw = R.from_rotvec(yaw * np.array([0, 1, 0]), degrees=True).as_matrix()
        world_coord_rotation = np.matmul(rotation, rotation_yaw)

        aligned_frames = align.process(framesD435)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue
        if time.time() - last_time_3d_image > 0.5:
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            last_time_3d_image = time.time()
        
        yaw_increment = yaw - yaw_previous
        car_in_world_coord_x_increment = car_in_world_coord_x - car_in_world_coord_x_previous
        car_in_world_coord_z_increment = car_in_world_coord_z - car_in_world_coord_z_previous
        map = rotate_and_move_map(map, yaw_increment, car_in_world_coord_z_increment * 100, -car_in_world_coord_x_increment * 100)

        visible_cone = np.array([[213, 242], [187, 242], [0, 0], [400, 0]], np.int32)
        visible_cone = visible_cone.reshape((-1, 1, 2))
        if time.time() - last_time_clear_map > 3: #map is cleared every 3 seconds to avoid old data
            cv2.fillPoly(map, [visible_cone], (100,100,100))
            last_time_clear_map=time.time()

        
        for x in range(0, realsenseW, 70):
            for y in range (0, realsenseH, 15):
                cx, cy, cz = pixel_to_car_coord(x, y)
                if cy > 0.4:					#ignore obstacles above car height
                    continue

                ### map 4x4m
                mx = int(cx * 100 + (mapW / 2) - 0) #3cm correction of camera position off center
                my = int((mapH - cz * 100) - 150) #car is 150cm from bottom of the map
    
                c = int(100 + cy * 100) # 100 is 0cm resolution 1cm
                if c < 0 or c > 255:
                    continue
                cv2.circle(map, (mx,my), 0, (c), thickness=-1, lineType=8, shift=0)
            
            
        
        for result in detector.detect(gray):
            x,y = result.center
            cx, cy, cz = pixel_to_car_coord(int(x),int(y))

            ### send target coordinates to redis server
            cwx, cwy, cwz = car_coord_to_world_coord(cx, cy, cz)
            target_coords_bytes = struct.pack('%sf' %3,* [cx, cy, cz])
            target_world_coords_bytes = struct.pack('%sf' %3,* [cwx, cwy, cwz])
            r.psetex('target_world_coords', 3000, target_world_coords_bytes) #target coordinates expire after xx milliseconds
            r.psetex('target_car_coords', 3000, target_coords_bytes) #target coordinates expire after xx milliseconds
            
        map_to_redis(r,map,'map')

        rotation_bytes = struct.pack('%sf' %3,* [pitch, roll, yaw])
        r.psetex('rotation', 1400, rotation_bytes) #yaw expire after xx milliseconds

        
        car_in_world_bytes = struct.pack('%sf' %3,* [car_in_world_coord_x, car_in_world_coord_y, car_in_world_coord_z])
        r.psetex('car_in_world', 1400, car_in_world_bytes) #yaw expire after xx milliseconds
        print(car_in_world_coord_x, car_in_world_coord_y, car_in_world_coord_z, yaw)
        print(time.time() - start_time)

        

except (RuntimeError,KeyboardInterrupt) as e:
    print("well shit",e)
    print("I ran for", time.time()-app_start_time, "seconds")
    #slam_map = pose_sensor.export_localization_map()
    #print(slam_map)
    #with open ('localization_map.map','w') as mapfile:
    #    for l in slam_map:
    #        mapfile.write(str(l)+"\n")

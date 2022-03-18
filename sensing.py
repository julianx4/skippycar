from logging import getLogRecordFactory
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

realsense_depth_W = 640
realsense_depth_H = 360

realsense_color_W = 640
realsense_color_H = 360


mapW = 400
mapH = 400

pi4 = False

camera_height=0.25 #mounting height of the depth camera vs. ground

detector = apriltag.Detector()

pipelineT265 = rs.pipeline()
configT265 = rs.config()
configT265.enable_device('908412110993') 
configT265.enable_stream(rs.stream.pose)

configT265.enable_stream(rs.stream.fisheye, 1)
configT265.enable_stream(rs.stream.fisheye, 2)

deviceT265 = configT265.resolve(pipelineT265).get_device()
pose_sensor = deviceT265.first_pose_sensor()
pose_sensor.set_option(rs.option.enable_map_preservation, 1)
pose_sensor.set_option(rs.option.enable_relocalization, 1)
pose_sensor.set_option(rs.option.enable_pose_jumping, 1)
pose_sensor.set_option(rs.option.enable_mapping, 1)



pipelineT265.start(configT265)
profileT265 = pipelineT265.get_active_profile()


pipelineD435 = rs.pipeline()
configD435 = rs.config()
configD435.enable_device('143322074867') 
configD435.enable_stream(rs.stream.depth, realsense_depth_W, realsense_depth_H, rs.format.z16, 15)
#configD435.enable_stream(rs.stream.infrared, 1, 640, 360, rs.format.y8, 30)
configD435.enable_stream(rs.stream.color, realsense_color_W, realsense_color_H, rs.format.bgr8, 15)

pipeline_wrapper = rs.pipeline_wrapper(pipelineD435)
pipeline_profile = configD435.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
#device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    exit(0)

profileD435 = pipelineD435.start(configD435)

depth_scale = profileD435.get_device().first_depth_sensor().get_depth_scale()
depth_min = 0.2 #meter
depth_max = 10.0 #meter

stream_profile_fish = profileT265.get_stream(rs.stream.fisheye, 1)
intrinsics_fish = stream_profile_fish.as_video_stream_profile().get_intrinsics()

stream_profile_depth = profileD435.get_stream(rs.stream.depth)
intrinsics_depth = stream_profile_depth.as_video_stream_profile().get_intrinsics()

stream_profile_color = profileD435.get_stream(rs.stream.color)
intrinsics_color = stream_profile_color.as_video_stream_profile().get_intrinsics()

depth_to_color_extrinsics =  profileD435.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(profileD435.get_stream(rs.stream.color))
color_to_depth_extrinsics =  profileD435.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(profileD435.get_stream(rs.stream.depth))

depth_frame = None

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

def color_pixel_to_depth_pixel(x, y, cam):
    if cam == "D435":
        intrinsics_detect = intrinsics_color
    else:
        intrinsics_detect = intrinsics_fish
    depthx, depthy = rs.rs2_project_color_pixel_to_depth_pixel(
        depth_frame.get_data(), depth_scale, depth_min, depth_max, 
        intrinsics_depth, intrinsics_detect, depth_to_color_extrinsics, 
        color_to_depth_extrinsics, [x,y])
    if depthx > realsense_depth_W: #somehow the number sometimes goes into the > 10e+30 range
        depthx = realsense_depth_W - 1
    if depthy > realsense_depth_H:
        depthy = realsense_depth_H - 1
    print(depthx, depthy)
    return depthx, depthy
#I use depth_to_color_extrinsics and color_to_depth_extrinsics also as extrinsics for the fisheye cam. It's not correct, but the error doesn't matter for my application  


def pixel_to_car_coord(x, y):
    dist = depth_frame.get_distance(x, y)
    cx,cy,cz = rs.rs2_deproject_pixel_to_point(intrinsics_depth, [x, y], dist)
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
last_time_clear_visible_cone = time.time()
car_in_world_coord_z = 0
car_in_world_coord_x = 0
start_time=time.time()
last_time_pipe_restart=time.time()
yaw_temp = 0
car_in_world_coord_x_temp = 0
car_in_world_coord_y_temp = 0
car_in_world_coord_z_temp = 0
app_start_time = time.time()
last_detect = time.time()
try:
    while True:
        start_time=time.time()
        car_in_world_coord_x_previous = car_in_world_coord_x
        car_in_world_coord_z_previous = car_in_world_coord_z
        yaw_previous = yaw

                
        framesT265 = pipelineT265.wait_for_frames(1000)
        framesD435 = pipelineD435.wait_for_frames()
        
        pose = framesT265.get_pose_frame()

        
        if pose:
            data = pose.get_pose_data()
            #print(data.rotation, data.translation)
            w = data.rotation.w
            x = -data.rotation.z
            y = data.rotation.x
            z = -data.rotation.y
            speedx = data.velocity.x
            speedy = data.velocity.y
            speedz = data.velocity.z

            car_in_world_coord_x = data.translation.x + car_in_world_coord_x_temp
            car_in_world_coord_y = data.translation.y + car_in_world_coord_y_temp
            car_in_world_coord_z = -data.translation.z + car_in_world_coord_z_temp

            pitch =  (-m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi) + 1.3; #1.3 degree misalignment between T265 tracking camera and D435 depth camera
            roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi 
            yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi + yaw_temp
            if time.time() - last_time_pipe_restart > 30 and pi4:
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
            speedx = 0
            speedy = 0
            speedz = 0
            
        speed = m.sqrt(speedx*speedx + speedy*speedy + speedz*speedz)

        rotation_roll = R.from_rotvec(roll * np.array([0, 0, 1]), degrees=True).as_matrix()
        rotation_pitch = R.from_rotvec(pitch * np.array([1, 0, 0]), degrees=True).as_matrix()
        rotation=np.matmul(rotation_roll, rotation_pitch)

        rotation_yaw = R.from_rotvec(yaw * np.array([0, 1, 0]), degrees=True).as_matrix()
        world_coord_rotation = np.matmul(rotation, rotation_yaw)

        color_frame_D435 = framesD435.get_color_frame()
        #color_frame = framesD435.get_infrared_frame(1)
        frame_T265 = framesT265.get_fisheye_frame(1)
        depth_frame = framesD435.get_depth_frame()

        if not frame_T265 or not depth_frame:
            continue

        
        color_image_D435 = np.asanyarray(color_frame_D435.get_data())
        gray_D435 = cv2.cvtColor(color_image_D435, cv2.COLOR_BGR2GRAY)

        last_time_3d_image = time.time()
        
        yaw_increment = yaw - yaw_previous
        car_in_world_coord_x_increment = car_in_world_coord_x - car_in_world_coord_x_previous
        car_in_world_coord_z_increment = car_in_world_coord_z - car_in_world_coord_z_previous
        map = rotate_and_move_map(map, yaw_increment, car_in_world_coord_z_increment * 100, -car_in_world_coord_x_increment * 100)

        visible_cone = np.array([[213, 242], [187, 242], [0, 0], [400, 0]], np.int32)
        visible_cone = visible_cone.reshape((-1, 1, 2))
        if time.time() - last_time_clear_map > 3: #map is cleared every 3 seconds to avoid old data
            map = np.full((mapW,mapH,1),100, np.uint8)
            last_time_clear_map=time.time()

        if time.time() - last_time_clear_visible_cone > 0.5: #visible cone is cleared every 0.5 seconds to ensure moving obstacles are not considered
            cv2.fillPoly(map, [visible_cone], 100)
            last_time_clear_visible_cone=time.time()


        cv2.rectangle(map, (188,230),(212,241), 100, -1) #clear area directly in front of car to avoid wrong information
        for x in range(0, realsense_depth_W, 50): #70
            for y in range (0, int(realsense_depth_H), 10): #15
                cx, cy, cz = pixel_to_car_coord(x, y)
                if cy > 0.4:					#ignore obstacles above car height
                    continue

                ### map 4x4m
                mx = int(cx * 100 + (mapW / 2) - 1.5) #3cm correction of camera position off center
                my = int((mapH - cz * 100) - 150) #car is 150cm from bottom of the map
    
                c = int(100 + cy * 100) # 100 is 0cm resolution 1cm
                if c < 0 or c > 255:
                    continue
                cv2.circle(map, (mx,my), 0, (c), thickness=-1, lineType=8, shift=0)
            
        detected = False

        for result_D435 in detector.detect(gray_D435):
            x,y = result_D435.center
            x,y = color_pixel_to_depth_pixel(x, y, "D435")
            r.psetex('log_detect_cam', 3000, "D435") 
            detected = True
        
        if not detected:
            ttttt = time.time()
            gray_T265 = np.asanyarray(frame_T265.get_data())
            gray_T265 = cv2.resize(gray_T265, (424, 400))
            for result in detector.detect(gray_T265):
                x,y = result.center
                x = x *2
                y = y *2
                x,y = color_pixel_to_depth_pixel(x, y, "T265")
                r.psetex('log_detect_cam', 3000, "T265") 
                detected = True
            #print("adsfasdofhasdofhasdofihsadfsa", time.time()-ttttt)

        last_detect = time.time()
        if detected:    
            cx, cy, cz = pixel_to_car_coord(int(x),int(y))
            cwx, cwy, cwz = car_coord_to_world_coord(cx, cy, cz)
            target_coords_bytes = struct.pack('%sf' %3,* [cx, cy, cz])
            target_world_coords_bytes = struct.pack('%sf' %3,* [cwx, cwy, cwz])
            r.psetex('target_world_coords', 3000, target_world_coords_bytes) #target coordinates expire after xx milliseconds
            r.psetex('target_car_coords', 3000, target_coords_bytes) #target coordinates expire after xx milliseconds

        
        detected = False
        map_to_redis(r,map,'map')

        rotation_bytes = struct.pack('%sf' %3,* [pitch, roll, yaw])
        r.psetex('rotation', 1400, rotation_bytes)

        
        car_in_world_bytes = struct.pack('%sf' %3,* [car_in_world_coord_x, car_in_world_coord_y, car_in_world_coord_z])
        r.psetex('car_in_world', 1400, car_in_world_bytes)
        r.psetex('current_speed', 1000, speed)
        r.psetex('log_uptime',1000, time.time() - app_start_time)
        r.psetex('log_sensing_running', 1000, "on")
        #print(car_in_world_coord_x, car_in_world_coord_y, car_in_world_coord_z, yaw)
        sensing_time = time.time() - start_time           
        r.psetex('log_sensing_time',1000, sensing_time)
        #print("sensing time", sensing_time)

        

except (RuntimeError,KeyboardInterrupt) as e:
    print("well shit",e)
    print("I ran for", time.time()-app_start_time, "seconds")


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
import curved_paths_coords as pc

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

#controllable variables
def rget_and_float(name, default = None):
    output = r.get(name)
    if output == None:
        return default
    else:
        return float(output)

target_memory_time = int(rget_and_float('target_memory_time', 1000))
tap_target_memory_time = int(rget_and_float('tap_target_memory_time', 1000))
map_base_height = int(rget_and_float('map_base_height', 100))
map_clear_all_interval = rget_and_float('map_clear_all_interval', 3)
map_clear_visible_cone_interval = rget_and_float('map_clear_visible_cone_interval', 0.5)
depth_pixel_horizontal_raster = int(rget_and_float('depth_pixel_horizontal_raster', 50))
depth_pixel_vertical_raster = int(rget_and_float('depth_pixel_vertical_raster', 10))
ignore_above_height = rget_and_float('ignore_above_height', 0.4)
square_range = int(rget_and_float('square_range', 6))
draw_grid = int(rget_and_float('draw_grid', 1))
draw_obstacles = int(rget_and_float('draw_obstacles', 1))
max_climb_height = rget_and_float('max_climb_height', 10)
#----

realsense_depth_W = 640
realsense_depth_H = 360

realsense_color_W = 640
realsense_color_H = 360


mapW = 400
mapH = 400

font = cv2.FONT_HERSHEY_SIMPLEX

pi4 = False

camera_height = 0.23 #mounting height of the depth camera vs. ground

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
    redis.set(name,encoded)
    #print(len(encoded))

def rotate_and_move_map(map, angle, up, right):
    image_center = tuple(np.array(map.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    move_mat = np.float32([
	[1, 0, right],
	[0, 1, up]])
    result_rot = cv2.warpAffine(map, rot_mat, map.shape[1::-1], flags=cv2.INTER_LINEAR, borderValue=(map_base_height,map_base_height,map_base_height))
    result = cv2.warpAffine(result_rot, move_mat, (result_rot.shape[1], result_rot.shape[0]), flags=cv2.INTER_LINEAR, borderValue=(map_base_height,map_base_height,map_base_height))
    return result

def color_pixel_to_depth_pixel(x, y, cam):
    if cam == "D435":
        intrinsics_detect = intrinsics_color
    else:
        intrinsics_detect = intrinsics_fish
        if x > 649:
            x = 649
        if x < 189:
            x = 189
        if y < 255:
            y = 255
        
    depthx, depthy = rs.rs2_project_color_pixel_to_depth_pixel(
        depth_frame.get_data(), depth_scale, depth_min, depth_max, 
        intrinsics_depth, intrinsics_detect, depth_to_color_extrinsics, 
        color_to_depth_extrinsics, [x,y])
    #if cam != "D435":
    #    print("depth: ", depthx, depthy)
    #    print("color: ", x,y)

    if depthx > realsense_depth_W: #somehow the number sometimes goes into the > 10e+30 range
        depthx = realsense_depth_W - 1
    if depthy > realsense_depth_H:
        depthy = realsense_depth_H - 1
    #print(depthx, depthy)
    return depthx, depthy
#I use depth_to_color_extrinsics and color_to_depth_extrinsics also as extrinsics for the fisheye cam. It's not correct, but the error doesn't matter for my application  


def pixel_to_car_coord(x, y):
    dist = depth_frame.get_distance(x, y)
    cx,cy,cz = rs.rs2_deproject_pixel_to_point(intrinsics_depth, [x, y], dist)
    c=np.array([cx,cy,cz])
    cx,cy,cz=np.matmul(rotation,c)
    cy = camera_height - cy #car_coords are floor level
    return cx, cy, cz

def car_coord_to_world_coord(x, y, z):
    c=np.array([x,y,z])
    cx,cy,cz=np.matmul(rotation_yaw,c)
    cx = car_in_world_coord_x + cx
    cy = car_in_world_coord_y + cy
    cz = car_in_world_coord_z + cz
    return cx, cy, cz

def draw_path_costs_on_image(image):
    for path in range(0, 11):
        path_cost = rget_and_float('path_cost'+str(path), 0)
        if path > 5:
            path_lookup = path - 5
            l = -1
        else:
            path_lookup = path
            l = 1
        #print('path_cost'+str(path), " ", path_cost)
        
        cv2.putText(image, str(int(path_cost)), (300 + 50 * l * path_lookup, 140), font, 0.4, (255,255,255), 1)

def draw_path_on_image(image):
    path_received = r.get('path')
    if path_received is None:
        return    
    elif int(path_received) == -1:
        return

    path = int(path_received)
    if path > 5:
        path_lookup = path - 5
        l = -1
    else:
        path_lookup = path
        l = 1

    x0_previous = pc.paths[0]['coords'][0][0] / 1000
    y0_previous = pc.paths[0]['coords'][0][1] / 1000
    x1_previous = pc.paths[0]['coords'][0][2] / 1000
    y1_previous = pc.paths[0]['coords'][0][3] / 1000    
    
    
    x0_previous, y0_previous = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([x0_previous, camera_height, y0_previous], rotation))
    x1_previous, y1_previous = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([x1_previous, camera_height, y1_previous], rotation))
    
    x0_previous = int(x0_previous)
    x1_previous = int(x1_previous)
    y0_previous = int(y0_previous)
    y1_previous = int(y1_previous)

    for square in range(0, square_range):
        square_height = rget_and_float('square_height'+str(square), 0) / 100 #cm to m


        x0 = l * pc.paths[path_lookup]['coords'][square][0] / 1000
        y0 = pc.paths[path_lookup]['coords'][square][1] / 1000
        x1 = l * pc.paths[path_lookup]['coords'][square][2] / 1000
        y1 = pc.paths[path_lookup]['coords'][square][3] / 1000

        x2 = l * pc.paths[path_lookup]['coords'][square + 1][0] / 1000
        y2 = pc.paths[path_lookup]['coords'][square + 1][1] / 1000
        x3 = l * pc.paths[path_lookup]['coords'][square + 1][2] / 1000
        y3 = pc.paths[path_lookup]['coords'][square + 1][3] / 1000

        x0, y0 = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([x0, camera_height - square_height, y0], rotation))
        x1, y1 = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([x1, camera_height - square_height, y1], rotation))
        x2, y2 = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([x2, camera_height - square_height, y2], rotation))
        x3, y3 = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([x3, camera_height - square_height, y3], rotation))
        x0 = int(x0)
        x1 = int(x1)
        x2 = int(x2)
        x3 = int(x3)
        y0 = int(y0)
        y1 = int(y1)
        y2 = int(y2)
        y3 = int(y3)
        
        cv2.line(image, (x0_previous, y0_previous), (x0, y0), (200, 200, 200), thickness=3)
        cv2.line(image, (x1_previous, y1_previous), (x1, y1), (200, 200, 200), thickness=3)

        x0_previous, y0_previous = x2, y2
        x1_previous, y1_previous = x3, y3 

        poly = np.array([[x0,y0],[x1,y1],[x3,y3],[x2,y2]])
        poly = poly.reshape((-1, 1, 2))
        cv2.polylines(image,[poly],True,(255,255,255),2)

map = np.full((mapW,mapH,1), map_base_height, np.uint8)
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
        tap_target_memory_time = int(rget_and_float('tap_target_memory_time', 1000))
        target_memory_time = int(rget_and_float('target_memory_time', 1000))
        map_base_height = int(rget_and_float('map_base_height', 100))
        map_clear_all_interval = rget_and_float('map_clear_all_interval', 3)
        map_clear_visible_cone_interval = rget_and_float('map_clear_visible_cone_interval', 0.5)
        depth_pixel_horizontal_raster = int(rget_and_float('depth_pixel_horizontal_raster', 50))
        depth_pixel_vertical_raster = int(rget_and_float('depth_pixel_vertical_raster', 10))
        ignore_above_height = rget_and_float('ignore_above_height', 0.4)        
        square_range = int(rget_and_float('square_range', 6))
        draw_grid = int(rget_and_float('draw_grid', 1))
        draw_obstacles = int(rget_and_float('draw_obstacles', 1))
        max_climb_height = rget_and_float('max_climb_height', 10)

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

            pitch =  (-m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi) + 1.95; #1.3 degree misalignment between T265 tracking camera and D435 depth camera
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
                map = np.full((mapW,mapH,1),map_base_height, np.uint8)
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
        rotation = np.matmul(rotation_roll, rotation_pitch)

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
        if time.time() - last_time_clear_map > map_clear_all_interval: #map is cleared every x seconds to avoid old data
            map = np.full((mapW,mapH,1),map_base_height, np.uint8)
            last_time_clear_map=time.time()

        if time.time() - last_time_clear_visible_cone > map_clear_visible_cone_interval: #visible cone is cleared every x seconds to ensure moving obstacles are not considered
            cv2.fillPoly(map, [visible_cone], map_base_height)
            last_time_clear_visible_cone=time.time()


        cv2.rectangle(map, (188,230),(212,241), 100, -1) #clear area directly in front of car to avoid wrong information

        detected = False
        tap_coord_X = r.get("tap_coord_X")
        tap_coord_Y = r.get("tap_coord_Y")
        tap_target = False
        if tap_coord_X is not None:
            detected = True
            tap_target = True
            tap_coord_X, tap_coord_Y = color_pixel_to_depth_pixel(int(tap_coord_X), int(tap_coord_Y), "D435")

        for result_D435 in detector.detect(gray_D435):
            x,y = result_D435.center
            #cv2.circle(color_image_D435, (int(x),int(y)), 10, (255,255,0), thickness=2, lineType=8, shift=0)
            x,y = color_pixel_to_depth_pixel(x, y, "D435")
            r.psetex('log_detect_cam', 3000, "D435") 
            
            detected = True
        
        if not detected:
            ttttt = time.time()
            gray_T265 = np.asanyarray(frame_T265.get_data())
            gray_T265 = cv2.resize(gray_T265, (424, 400))
            for result in detector.detect(gray_T265):
                x,y = result.center
                x = x * 2
                y = y * 2
 
                
                print("before: ", x,y)
                x,y = color_pixel_to_depth_pixel(x, y, "T265")
                if x > 1 and y > 1:
                    r.psetex('log_detect_cam', 3000, "T265")
                    detected = True
                    print("after: ", x,y)
                    
            

        last_detect = time.time()
        if detected:    
            if not tap_target:
                cx, cy, cz = pixel_to_car_coord(int(x),int(y))
                cwx, cwy, cwz = car_coord_to_world_coord(cx, cy, cz)
            else:
                cx, cy, cz = pixel_to_car_coord(int(tap_coord_X),int(tap_coord_Y))
                cwx, cwy, cwz = car_coord_to_world_coord(cx, cy, cz)
                target_memory_time = tap_target_memory_time

            target_coords_bytes = struct.pack('%sf' %3,* [cx, cy, cz])
            target_world_coords_bytes = struct.pack('%sf' %3,* [cwx, cwy, cwz])
            tagx, tagy = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([cx, camera_height - cy, cz], rotation))
            r.psetex('target_world_coords', target_memory_time, target_world_coords_bytes) #target coordinates expire after xx milliseconds
            r.psetex('target_car_coords', target_memory_time, target_coords_bytes) #target coordinates expire after xx milliseconds
            tag_floor_x, tag_floor_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([cx, camera_height, cz], rotation))
            zero_car_x, zero_car_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([0, camera_height, 0.10], rotation))
            cv2.circle(color_image_D435, (int(tagx),int(tagy)), 10, (255,255,255), thickness=2, lineType=8, shift=0)
            cv2.circle(color_image_D435, (int(tag_floor_x),int(tag_floor_y)), 10, (255,255,255), thickness=2, lineType=8, shift=0)
            cv2.line(color_image_D435, (int(tag_floor_x), int(tag_floor_y)), (int(tagx), int(tagy)), (255,0,0), thickness = 2) 
            cv2.line(color_image_D435, (int(zero_car_x), int(zero_car_y)), (int(tag_floor_x), int(tag_floor_y)), (255,0,0), thickness = 5)
        detected = False



        for x in range(0, realsense_depth_W, depth_pixel_horizontal_raster): #70
            for y in range (int(realsense_depth_H/3), int(realsense_depth_H), depth_pixel_vertical_raster): #15
                cx, cy, cz = pixel_to_car_coord(x, y)
                if cy > ignore_above_height:					#ignore obstacles above car height
                    continue

                ### map 4x4m
                mx = int(cx * 100 + (mapW / 2) - 3) #3cm correction of camera position off center
                my = int((mapH - cz * 100) - 150) #car is 150cm from bottom of the map
    
                c = int(100 + cy * 100) # 100 is 0cm resolution 1cm
                if c < 0 or c > 255:
                    continue
                cv2.circle(map, (mx,my), 0, (c), thickness=-1, lineType=8, shift=0)

                if draw_obstacles == 1:
                    line_thickness = 2
                    if abs(cy) < 0.02 or cz == 0: #
                        continue
                    obstacle_top_x, obstacle_top_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([cx, -(cy - camera_height), cz + 0.01], rotation))                
                    obstacle_bottom_x, obstacle_bottom_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([cx, camera_height, cz + 0.01], rotation))
                 
                    obstacle_bottom_x = int(obstacle_bottom_x)
                    obstacle_top_x = int(obstacle_top_x)
                    obstacle_bottom_y = int(obstacle_bottom_y)
                    obstacle_top_y = int(obstacle_top_y)
                    

                    if abs(cy) > max_climb_height / 100:
                        
                        line_color = (0, 0, 255 - min(cz * 200, 255))
                    else:
                        line_color = (0, 255 - min(cz * 200, 255), 0)
                    #print(abs(cy))   

                    if -(cy - camera_height) < camera_height:      
                        cv2.line(color_image_D435, (obstacle_top_x - 10, obstacle_top_y + 10), (obstacle_top_x, obstacle_top_y), line_color, thickness = line_thickness)
                        cv2.line(color_image_D435, (obstacle_top_x + 10, obstacle_top_y + 10), (obstacle_top_x, obstacle_top_y), line_color, thickness = line_thickness)
                        
                    else:
                        cv2.line(color_image_D435, (obstacle_top_x - 10, obstacle_top_y - 10), (obstacle_top_x, obstacle_top_y), line_color, thickness = line_thickness)
                        cv2.line(color_image_D435, (obstacle_top_x + 10, obstacle_top_y - 10), (obstacle_top_x, obstacle_top_y), line_color, thickness = line_thickness)
                        
                    cv2.line(color_image_D435, (obstacle_top_x, obstacle_top_y), (obstacle_bottom_x, obstacle_bottom_y), line_color, thickness = line_thickness)
                    


                    


        if draw_grid == 1:
            forward_begin = 0.01
            forward_end = 3
            horizontal_begin = -2
            horizontal_end = 2
            grid_step = 0.25
            for horizontal_step in np.arange(horizontal_begin, horizontal_end, grid_step):

                line_begin_x, line_begin_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([horizontal_step, camera_height, forward_begin], rotation))
                line_end_x, line_end_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([horizontal_step, camera_height, forward_end], rotation))
                
                line_begin_x = int(line_begin_x)
                line_begin_y = int(line_begin_y)
                line_end_x = int(line_end_x)
                line_end_y = int(line_end_y)
                
                cv2.line(color_image_D435, (line_begin_x, line_begin_y), (line_end_x, line_end_y), (200,200,200), thickness=1)

            for forward_step in np.arange(forward_begin, forward_end, grid_step):
                line_begin_x, line_begin_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([horizontal_begin, camera_height, forward_step], rotation))
                line_end_x, line_end_y = rs.rs2_project_point_to_pixel(intrinsics_color, np.matmul([horizontal_end, camera_height, forward_step], rotation))
                
                line_begin_x = int(line_begin_x)
                line_begin_y = int(line_begin_y)
                line_end_x = int(line_end_x)
                line_end_y = int(line_end_y)
                
                cv2.line(color_image_D435, (line_begin_x, line_begin_y), (line_end_x, line_end_y), (200,200,200), thickness=1)

        map_to_redis(r,map,'map')

        draw_path_on_image(color_image_D435)
        draw_path_costs_on_image(color_image_D435)
        map_to_redis(r, color_image_D435,"D435_image")
        

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
    if time.time()-app_start_time < 2:
        deviceT265.hardware_reset()


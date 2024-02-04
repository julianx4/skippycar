import redis
import struct
import pyrealsense2 as rs
import apriltag
import cv2
import numpy as np
import time
import math as m
from scipy.spatial.transform import Rotation as R
import curved_paths_coords as pc

class RedisManager:
    def __init__(self):
        self.r = redis.Redis(host='localhost', port=6379, db=0)
    
    def get_float(self, name, default=None):
        output = self.r.get(name)
        if output is None:
            return default
        else:
            return float(output)

    def set_data(self, name, data, expiry=None):
        if expiry:
            self.r.psetex(name, expiry, data)
        else:
            self.r.set(name, data)

    def get_data(self, name):
        output = self.r.get(name)
        if output == None:
            return None
        else:
            return float(output)

    def map_image_to_redis(self, name, array):
        h, w = array.shape[:2]
        shape = struct.pack('>II', h, w)
        encoded = shape + array.tobytes()
        self.r.set(name, encoded)

class RealSenseManager:
    def __init__(self):
        self.realsense_depth_W = 1280 #640
        self.realsense_depth_H = 720 # 360
        self.realsense_color_W = 1280
        self.realsense_color_H = 720

        self.pipelineT265 = rs.pipeline()
        self.configT265 = rs.config()
        self.configT265.enable_device('908412110993') 
        self.configT265.enable_stream(rs.stream.pose)

        self.configT265.enable_stream(rs.stream.fisheye, 1)
        self.configT265.enable_stream(rs.stream.fisheye, 2)

        self.deviceT265 = self.configT265.resolve(self.pipelineT265).get_device()
        self.pose_sensor = self.deviceT265.first_pose_sensor()
        self.pose_sensor.set_option(rs.option.enable_map_preservation, 1)
        self.pose_sensor.set_option(rs.option.enable_relocalization, 1)
        self.pose_sensor.set_option(rs.option.enable_pose_jumping, 1)
        self.pose_sensor.set_option(rs.option.enable_mapping, 1)

        self.pipelineT265.start(self.configT265)
        self.profileT265 = self.pipelineT265.get_active_profile()

        self.pipelineD435 = rs.pipeline()
        self.configD435 = rs.config()
        self.configD435.enable_device('143322074867')
        self.configD435.enable_stream(rs.stream.depth, self.realsense_depth_W, self.realsense_depth_H, rs.format.z16, 15)
        self.configD435.enable_stream(rs.stream.color, self.realsense_color_W, self.realsense_depth_H, rs.format.bgr8, 15)

        self.pipelineD435.start(self.configD435)

        self.profileD435 = self.pipelineD435.get_active_profile()
        self.depth_scale = self.profileD435.get_device().first_depth_sensor().get_depth_scale()
        self.depth_min = 0.2  # meter
        self.depth_max = 10.0  # meter

        self.stream_profile_fish = self.profileT265.get_stream(rs.stream.fisheye, 1)
        self.intrinsics_fish = self.stream_profile_fish.as_video_stream_profile().get_intrinsics()

        stream_profile_depth = self.profileD435.get_stream(rs.stream.depth)
        self.intrinsics_depth = stream_profile_depth.as_video_stream_profile().get_intrinsics()

        stream_profile_color = self.profileD435.get_stream(rs.stream.color)
        self.intrinsics_color = stream_profile_color.as_video_stream_profile().get_intrinsics()

        self.depth_to_color_extrinsics =  self.profileD435.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(self.profileD435.get_stream(rs.stream.color))
        self.color_to_depth_extrinsics =  self.profileD435.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(self.profileD435.get_stream(rs.stream.depth))

        self.pose = None
        self.depth_frame = None
        self.color_image_D435 = None
        self.gray_image_D435 = None
        self.bw_image_T265 = None
        self.rotation = None
        self.speed = None

        self.yaw = 0
        self.car_in_world_coord_z = 0
        self.car_in_world_coord_x = 0
        self.yaw_previous = 0
        self.rotation_yaw = None
        self.car_in_world_coord_x_previous = 0
        self.car_in_world_coord_z_previous = 0

        self.camera_height = 0.20  # mounting height of the depth camera vs. ground
    
    def get_frames(self):
        self.framesT265 = self.pipelineT265.wait_for_frames(1000)
        self.framesD435 = self.pipelineD435.wait_for_frames()

    def get_pose(self):
        self.pose = self.framesT265.get_pose_frame()
    
    def get_image_D435(self):
        color_frame = self.framesD435.get_color_frame()
        self.color_image_D435 = np.asanyarray(color_frame.get_data())
        self.gray_image_D435 = cv2.cvtColor(self.color_image_D435, cv2.COLOR_BGR2GRAY)
    
    def get_depth_frame_D435(self):
        self.depth_frame = self.framesD435.get_depth_frame()
    
    def get_bw_image_T265(self):
        fisheye1_frame = self.framesT265.get_fisheye_frame(1)
        #fisheye2_frame = self.configT265.get_fisheye_frame(2)
        self.bw_image_T265 = np.asanyarray(fisheye1_frame.get_data())
        #image2 = np.asanyarray(fisheye2_frame.get_data())

    def pixel_to_car_coord(self, x, y):
        dist = self.depth_frame.get_distance(x, y)
        cx,cy,cz = rs.rs2_deproject_pixel_to_point(self.intrinsics_depth, [x, y], dist)
        c=np.array([cx,cy,cz])
        cx,cy,cz=np.matmul(self.rotation,c)
        cy = self.camera_height - cy #car_coords are floor level
        return cx, cy, cz

    def car_coord_to_world_coord(self,x, y, z):
        c = np.array([x,y,z])
        cx, cy, cz=np.matmul(self.rotation_yaw,c)
        cx = self.car_in_world_coord_x + cx
        cy = self.car_in_world_coord_y + cy
        cz = self.car_in_world_coord_z + cz
        return cx, cy, cz

    def world_coord_to_car_coord(self, world_x, world_y, world_z):
        translated_x = world_x - self.car_in_world_coord_x
        translated_y = world_y - self.car_in_world_coord_y
        translated_z = world_z - self.car_in_world_coord_z

        point = np.array([translated_x, translated_y, translated_z])

        inverse_rotation_yaw = np.linalg.inv(self.rotation_yaw)

        car_coords = np.matmul(inverse_rotation_yaw, point)
        return car_coords[0], car_coords[1], car_coords[2]

    def get_rotation(self):
        self.car_in_world_coord_x_previous = self.car_in_world_coord_x
        self.car_in_world_coord_z_previous = self.car_in_world_coord_z

        pose = self.pose

        data = pose.get_pose_data()
        #print(data.rotation, data.translation)
        w = data.rotation.w
        x = -data.rotation.z
        y = data.rotation.x
        z = -data.rotation.y
        speedx = data.velocity.x
        speedy = data.velocity.y
        speedz = data.velocity.z

        self.car_in_world_coord_x = data.translation.x
        self.car_in_world_coord_y = data.translation.y
        self.car_in_world_coord_z = -data.translation.z

        pitch = (-m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi) + 1.95; #1.3 degree misalignment between T265 tracking camera and D435 depth camera
        roll = m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi 
        self.yaw = m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi
            
        self.speed = m.sqrt(speedx ** 2 + speedy ** 2 + speedz ** 2)

        rotation_roll = R.from_rotvec(roll * np.array([0, 0, 1]), degrees=True).as_matrix()
        rotation_pitch = R.from_rotvec(pitch * np.array([1, 0, 0]), degrees=True).as_matrix()
        self.rotation = np.matmul(rotation_roll, rotation_pitch)

        self.rotation_yaw = R.from_rotvec(self.yaw * np.array([0, 1, 0]), degrees=True).as_matrix()
        self.world_coord_rotation = np.matmul(self.rotation, self.rotation_yaw)

        self.car_in_world_coord_x_increment = self.car_in_world_coord_x - self.car_in_world_coord_x_previous
        self.car_in_world_coord_z_increment = self.car_in_world_coord_z - self.car_in_world_coord_z_previous

    def get_yaw_increment(self):
        self.yaw_increment = self.yaw - self.yaw_previous
        self.yaw_previous = self.yaw

    def color_pixel_to_depth_pixel(self, x, y, cam):
        if cam == "D435":
            intrinsics_detect = self.intrinsics_color
        else:
            intrinsics_detect = self.intrinsics_fish
            if x > rsm.realsense_color_W:
                x = rsm.realsense_color_W
            if x < 0:
                x = 0
            if y > rsm.realsense_color_H:
                y = rsm.realsense_color_H
            if y < 0:
                y = 0
            
        depthx, depthy = rs.rs2_project_color_pixel_to_depth_pixel(
                    self.depth_frame.get_data(), 
                    self.depth_scale, 
                    self.depth_min, 
                    self.depth_max, 
                    self.intrinsics_depth, 
                    intrinsics_detect, 
                    self.depth_to_color_extrinsics, 
                    self.color_to_depth_extrinsics, 
                    [x,y])

        if depthx > self.realsense_depth_W: #somehow the number sometimes goes into the > 10e+30 range
            depthx = self.realsense_depth_W - 1
        if depthy > self.realsense_depth_H:
            depthy = self.realsense_depth_H - 1
        return depthx, depthy
        #I use depth_to_color_extrinsics and color_to_depth_extrinsics also as extrinsics for the fisheye cam. It's not correct, but the error doesn't matter for my application  

    def draw_path_on_image(self, square_range):
        square_range = square_range
        path_received = rdm.get_data('path')
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
        
        
        x0_previous, y0_previous = rs.rs2_project_point_to_pixel(self.intrinsics_color, np.matmul([x0_previous, self.camera_height, y0_previous], self.rotation))
        x1_previous, y1_previous = rs.rs2_project_point_to_pixel(self.intrinsics_color, np.matmul([x1_previous, self.camera_height, y1_previous], self.rotation))
        
        x0_previous = int(x0_previous)
        x1_previous = int(x1_previous)
        y0_previous = int(y0_previous)
        y1_previous = int(y1_previous)

        for square in range(0, square_range):
            square_height = rdm.get_data('square_height'+str(square)) / 100 #cm to m


            x0 = l * pc.paths[path_lookup]['coords'][square][0] / 1000
            y0 = pc.paths[path_lookup]['coords'][square][1] / 1000
            x1 = l * pc.paths[path_lookup]['coords'][square][2] / 1000
            y1 = pc.paths[path_lookup]['coords'][square][3] / 1000

            x2 = l * pc.paths[path_lookup]['coords'][square + 1][0] / 1000
            y2 = pc.paths[path_lookup]['coords'][square + 1][1] / 1000
            x3 = l * pc.paths[path_lookup]['coords'][square + 1][2] / 1000
            y3 = pc.paths[path_lookup]['coords'][square + 1][3] / 1000

            x0, y0 = rs.rs2_project_point_to_pixel(self.intrinsics_color, np.matmul([x0, self.camera_height - square_height, y0], self.rotation))
            x1, y1 = rs.rs2_project_point_to_pixel(self.intrinsics_color, np.matmul([x1, self.camera_height - square_height, y1], self.rotation))
            x2, y2 = rs.rs2_project_point_to_pixel(self.intrinsics_color, np.matmul([x2, self.camera_height - square_height, y2], self.rotation))
            x3, y3 = rs.rs2_project_point_to_pixel(self.intrinsics_color, np.matmul([x3, self.camera_height - square_height, y3], self.rotation))
            x0 = int(x0)
            x1 = int(x1)
            x2 = int(x2)
            x3 = int(x3)
            y0 = int(y0)
            y1 = int(y1)
            y2 = int(y2)
            y3 = int(y3)
            
            cv2.line(self.color_image_D435, (x0_previous, y0_previous), (x0, y0), (200, 200, 200), thickness=3)
            cv2.line(self.color_image_D435, (x1_previous, y1_previous), (x1, y1), (200, 200, 200), thickness=3)

            x0_previous, y0_previous = x2, y2
            x1_previous, y1_previous = x3, y3 

            poly = np.array([[x0,y0],[x1,y1],[x3,y3],[x2,y2]])
            poly = poly.reshape((-1, 1, 2))
            cv2.polylines(self.color_image_D435,[poly],True,(255,255,255),2)

    def update_realsense_data(self):
        self.get_frames()
        self.get_pose()
        self.get_image_D435()
        self.get_bw_image_T265()
        self.get_depth_frame_D435()
        self.get_rotation()

    
class AprilTagDetector:
    def __init__(self, realsensemanager, redismanager):
        self.detector = apriltag.Detector()
        self.rsm = realsensemanager
        self.rdm = redismanager

        self.target_world_coords = None
        self.target_car_coords = None   

    def detect_tags(self):
        x = y = 1000
        detected = False
        tags = self.detector.detect(self.rsm.gray_image_D435)
        for tag in tags:
            x, y = tag.center
            x, y = rsm.color_pixel_to_depth_pixel(x, y, "D435")
            rdm.set_data('log_detect_cam', "D435", 3000)
            detected = True
        if tags == []:
            tags = self.detector.detect(self.rsm.bw_image_T265)
            for tag in tags:
                x, y = tag.center
                x, y = rsm.color_pixel_to_depth_pixel(x, y, "T265")
                rdm.set_data('log_detect_cam', "T265", 3000)
                detected = True

        if x > rsm.realsense_color_W or y > rsm.realsense_color_H or x < 0 or y < 0:
            detected = False 

        if detected:
            cx, cy, cz = rsm.pixel_to_car_coord(int(x),int(y))
            cwx, cwy, cwz = rsm.car_coord_to_world_coord(cx, cy, cz)
            self.target_world_coords = [cwx, cwy, cwz]
            tagx, tagy = rs.rs2_project_point_to_pixel(self.rsm.intrinsics_color, np.matmul([cx, self.rsm.camera_height - cy, cz], self.rsm.rotation))
            tag_floor_x, tag_floor_y = rs.rs2_project_point_to_pixel(self.rsm.intrinsics_color, np.matmul([cx, self.rsm.camera_height, cz], self.rsm.rotation))
            zero_car_x, zero_car_y = rs.rs2_project_point_to_pixel(self.rsm.intrinsics_color, np.matmul([0, self.rsm.camera_height, 0.10], self.rsm.rotation))
            cv2.circle(self.rsm.color_image_D435, (int(tagx),int(tagy)), 10, (255,255,255), thickness=2, lineType=8, shift=0)
            cv2.circle(self.rsm.color_image_D435, (int(tag_floor_x),int(tag_floor_y)), 10, (255,255,255), thickness=2, lineType=8, shift=0)
            cv2.line(self.rsm.color_image_D435, (int(tag_floor_x), int(tag_floor_y)), (int(tagx), int(tagy)), (255,0,0), thickness = 2) 
            cv2.line(self.rsm.color_image_D435, (int(zero_car_x), int(zero_car_y)), (int(tag_floor_x), int(tag_floor_y)), (255,0,0), thickness = 5)

class MapManager:
    def __init__(self, width, height, base_height, realsensemanager, redismanager):
        self.mapW = width
        self.mapH = height
        self.map_base_height = base_height
        # self.map = np.full((self.mapW, self.mapH, 1), self.map_base_height, np.uint8)
        self.map = np.zeros((self.mapW, self.mapH, 2), dtype=np.uint8)  # Added layer for occupancy probability
        self.map[:, :, 0] = self.map_base_height  # Height layer initialization
        self.last_update_times = np.full((self.mapW, self.mapH), time.time())

        self.last_time_clear_map = time.time()
        self.last_time_clear_visible_cone = time.time()
        self.realsensemanager = realsensemanager
        self.redismanager = redismanager

        self.car_position_on_map = 150
        self.camera_height = realsensemanager.camera_height

        self.update_variables()

        self.create_visible_cone_mask()

    def update_variables(self):
        self.target_memory_time = int(self.redismanager.get_float('target_memory_time', 1000))
        self.tap_target_memory_time = int(self.redismanager.get_float('tap_target_memory_time', 1000))
        self.map_base_height = int(self.redismanager.get_float('map_base_height', 100))
        self.map_clear_all_interval = self.redismanager.get_float('map_clear_all_interval', 3)
        self.map_clear_visible_cone_interval = self.redismanager.get_float('map_clear_visible_cone_interval', 0.5)
        self.depth_pixel_horizontal_raster = int(self.redismanager.get_float('depth_pixel_horizontal_raster', 5)) #50
        self.depth_pixel_vertical_raster = int(self.redismanager.get_float('depth_pixel_vertical_raster', 1)) #10
        self.ignore_above_height = self.redismanager.get_float('ignore_above_height', 0.4)
        self.square_range = int(self.redismanager.get_float('square_range', 6))
        self.draw_grid = int(self.redismanager.get_float('draw_grid', 1))
        self.draw_obstacles = int(self.redismanager.get_float('draw_obstacles', 1))
        self.max_climb_height = self.redismanager.get_float('max_climb_height', 10)

    def create_visible_cone_mask(self):
        # Create an empty mask with the same dimensions as the map
        mask = np.zeros((self.mapW, self.mapH), dtype=np.uint8)
        
        # Define the visible cone polygon
        visible_cone = np.array([[213, 242], [187, 242], [0, 0], [400, 0]], np.int32)
        
        # Fill the visible cone polygon with 1s in the mask
        cv2.fillPoly(mask, [visible_cone], 1)
        
        # Store the mask for later use
        self.visible_cone_mask = mask

    def rotate_and_move_map(self, angle, up, right):
        car_position_x = self.mapW // 2  # Assuming car is in the middle of the map horizontally
        car_position_y = self.mapH - self.car_position_on_map  # Car is 150cm from the bottom of the map
        rotation_center = (car_position_x, car_position_y)
        # Rotation matrix
        rot_mat = cv2.getRotationMatrix2D(rotation_center, angle, 1.0)
        
        # Perform rotation
        result_rot = cv2.warpAffine(self.map, rot_mat, self.map.shape[1::-1], flags=cv2.INTER_LINEAR, borderValue=(self.map_base_height, self.map_base_height, self.map_base_height))
        
        # Movement matrix (translation)
        move_mat = np.float32([
            [1, 0, right],
            [0, 1, up]])
        
        # Perform movement
        result = cv2.warpAffine(result_rot, move_mat, (result_rot.shape[1], result_rot.shape[0]), flags=cv2.INTER_LINEAR, borderValue=(self.map_base_height, self.map_base_height, self.map_base_height))
        
        self.map = result

    def clear_map(self, map_clear_all_interval, map_clear_visible_cone_interval):
        if time.time() - self.last_time_clear_map > map_clear_all_interval:  # map is cleared every x seconds to avoid old data
            self.map = np.full((self.mapW, self.mapH, 1), self.map_base_height, np.uint8)
            self.last_time_clear_map = time.time()

        if time.time() - self.last_time_clear_visible_cone > map_clear_visible_cone_interval:  # visible cone is cleared every x seconds to ensure moving obstacles are not considered
            visible_cone = np.array([[213, 242], [187, 242], [0, 0], [400, 0]], np.int32)
            visible_cone = visible_cone.reshape((-1, 1, 2))
            cv2.fillPoly(self.map, [visible_cone], self.map_base_height)
            self.last_time_clear_visible_cone = time.time()

    def clear_area_in_front_of_car(self):
        cv2.rectangle(self.map, (188, 230), (212, 241), self.map_base_height, -1)  # clear area directly in front of car to avoid wrong information

    def update_map_and_obstacles(self, depth_frame, max_climb_height, rotation, color_image_D435, occupancy_decrease_rate, occupancy_increase_rate):
        realsense_depth_W, realsense_depth_H = self.realsensemanager.realsense_depth_W, self.realsensemanager.realsense_depth_H
        depth_pixel_horizontal_raster, depth_pixel_vertical_raster = self.depth_pixel_horizontal_raster, self.depth_pixel_vertical_raster
        ignore_above_height = self.ignore_above_height
        camera_height = self.camera_height
        current_time = time.time()
        for x in range(0, realsense_depth_W, depth_pixel_horizontal_raster):
            for y in range(int(realsense_depth_H / 3), int(realsense_depth_H), depth_pixel_vertical_raster):
                dist = depth_frame.get_distance(x, y)
                cx, cy, cz = rs.rs2_deproject_pixel_to_point(self.realsensemanager.intrinsics_color, [x, y], dist)
                c = np.array([cx, cy, cz])
                cx, cy, cz = np.matmul(rotation, c)
                cy = camera_height - cy  # car_coords are floor level

                if cy > ignore_above_height:  # ignore obstacles above car height
                    continue

                mx = int(cx * 100 + (self.mapW / 2) - 3)  # 3cm correction of camera position off center
                my = int((self.mapH - cz * 100) - self.car_position_on_map)  # car is 150cm from bottom of the map

                if mx < 0 or mx >= self.mapW or my < 0 or my >= self.mapH:
                    continue  # Skip this point because it's outside the map

                # Update height and occupancy probability
                self.map[my, mx, 0] = int(100 + cy * 100)  # 100 is 0cm resolution 1cm

                occupancy_probability = self.map[my, mx, 1]
                if cy < ignore_above_height:  # If obstacle detected
                    # Increase occupancy probability
                    occupancy_probability = min(occupancy_probability + occupancy_increase_rate, 255)
                else:
                    # Decrease occupancy probability
                    occupancy_probability = max(occupancy_probability - occupancy_decrease_rate, 0)
                self.map[my, mx, 1] = occupancy_probability

                # Update the last update time for this cell
                self.last_update_times[my, mx] = current_time

                if self.draw_obstacles:
                    line_thickness = 2
                    if abs(cy) < 0.02 or cz == 0:  #
                        continue
                    obstacle_top_x, obstacle_top_y = rs.rs2_project_point_to_pixel(self.realsensemanager.intrinsics_color, np.matmul([cx, -(cy - camera_height), cz + 0.01], rotation))                
                    obstacle_bottom_x, obstacle_bottom_y = rs.rs2_project_point_to_pixel(self.realsensemanager.intrinsics_color, np.matmul([cx, camera_height, cz + 0.01], rotation))
                 
                    obstacle_bottom_x = int(obstacle_bottom_x)
                    obstacle_top_x = int(obstacle_top_x)
                    obstacle_bottom_y = int(obstacle_bottom_y)
                    obstacle_top_y = int(obstacle_top_y)
                    

                    if abs(cy) > max_climb_height / 100:
                        line_color = (0, 0, 255 - min(cz * 200, 255))
                    else:
                        line_color = (0, 255 - min(cz * 200, 255), 0)

                    if -(cy - camera_height) < camera_height:      
                        cv2.line(color_image_D435, (obstacle_top_x - 10, obstacle_top_y + 10), (obstacle_top_x, obstacle_top_y), line_color, thickness=line_thickness)
                        cv2.line(color_image_D435, (obstacle_top_x + 10, obstacle_top_y + 10), (obstacle_top_x, obstacle_top_y), line_color, thickness=line_thickness)
                    else:
                        cv2.line(color_image_D435, (obstacle_top_x - 10, obstacle_top_y - 10), (obstacle_top_x, obstacle_top_y), line_color, thickness=line_thickness)
                        cv2.line(color_image_D435, (obstacle_top_x + 10, obstacle_top_y - 10), (obstacle_top_x, obstacle_top_y), line_color, thickness=line_thickness)
                        
                    cv2.line(color_image_D435, (obstacle_top_x, obstacle_top_y), (obstacle_bottom_x, obstacle_bottom_y), line_color, thickness=line_thickness)


    def decay_occupancy_probabilities(self, decay_rate, max_decay):
        current_time = time.time()
        time_since_last_update = current_time - self.last_update_times
        
        # Calculate decay amount based on the time since the last update
        decay_amount = decay_rate * time_since_last_update
        decay_amount = np.clip(decay_amount, 0, max_decay)  # Limit the decay amount to max_decay
        
        # Ensure decay is only applied within the visible cone
        decay_mask = (self.visible_cone_mask == 1)
        time_since_last_update_masked = time_since_last_update * decay_mask  # Apply the mask
        decay_amount_masked = decay_rate * time_since_last_update_masked  # Calculate decay only for masked areas
        decay_amount_masked = np.clip(decay_amount_masked, 0, max_decay)  # Limit the decay amount to max_decay
        
        # Apply decay to the occupancy probabilities only within the visible cone
        self.map[:, :, 1] = np.clip(self.map[:, :, 1] - decay_amount_masked, 0, 255)

    def send_map_to_redis(self, redis_manager):
        # Create a modified version of the height map
        # Set the height to base_height if the occupancy probability is less than 50%
        modified_map = np.where(self.map[:, :, 1] < 30, self.map_base_height, self.map[:, :, 0])
        #modified_map = self.map[:, :, 1]
        # Convert the modified map to bytes and send to Redis
        redis_manager.map_image_to_redis('map', modified_map)


rsm = RealSenseManager()
rdm = RedisManager()
atd = AprilTagDetector(rsm, rdm)
mapc = MapManager(400, 400, 100, rsm, rdm)
app_start_time = time.time()
while True:
    loop_start_time = time.time()
    rsm.update_realsense_data()
    mapc.update_variables()
    
    rsm.get_rotation()
    
    
    mapc.update_map_and_obstacles(depth_frame = rsm.depth_frame, 
                                max_climb_height = 10,
                                rotation = rsm.rotation, 
                                color_image_D435 = rsm.color_image_D435,
                                occupancy_decrease_rate = 10, 
                                occupancy_increase_rate = 10
                                )
    mapc.decay_occupancy_probabilities(decay_rate = 0.01, 
                                      max_decay = 10)
    
    rsm.get_yaw_increment()
    mapc.rotate_and_move_map(rsm.yaw_increment, rsm.car_in_world_coord_z_increment * 100, -rsm.car_in_world_coord_x_increment * 100)
    mapc.send_map_to_redis(rdm)

    atd.detect_tags()
    if atd.target_world_coords is not None:
        target_world_x, target_world_y, target_world_z =  atd.target_world_coords
        target_car_x, target_car_y, target_car_z = rsm.world_coord_to_car_coord(target_world_x, target_world_y, target_world_z)
        rdm.set_data('target_car_coords', struct.pack('%sf' %3, target_car_x, target_car_y, target_car_z), 1000)
    rdm.set_data('log_uptime', time.time() - app_start_time, 1000)
    rdm.set_data('log_sensing_running', 'on', 1000)
    rdm.set_data('current_speed', rsm.speed)

    rsm.draw_path_on_image(mapc.square_range)
    rdm.map_image_to_redis('D435_image', rsm.color_image_D435)
    sensing_time = time.time() - loop_start_time
    rdm.set_data('log_sensing_time', sensing_time, 1000)

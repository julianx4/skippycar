


import pyrealsense2 as rs

import cv2
import numpy as np
import time
import math as m
from scipy.spatial.transform import Rotation as R


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

    def update_realsense_data(self):
        self.get_frames()
        self.get_pose()
        self.get_image_D435()
        self.get_bw_image_T265()
        self.get_depth_frame_D435()
        self.get_rotation()

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
        # Fetch the fisheye frame from the first camera
        fisheye_frame = self.framesT265.get_fisheye_frame(1)
        image = np.asanyarray(fisheye_frame.get_data())

        # Camera matrix K and distortion coefficients D
        K = np.array([[self.intrinsics_fish.fx, 0, self.intrinsics_fish.ppx],
                    [0, self.intrinsics_fish.fy, self.intrinsics_fish.ppy],
                    [0, 0, 1]])
        D = np.array(self.intrinsics_fish.coeffs[:4])

        # Identity matrix for rotation
        R = np.eye(3)
        
        # Image size
        size = (self.intrinsics_fish.width, self.intrinsics_fish.height)

        # Compute the optimal new camera matrix based on the free scaling parameter
        # Adjusting the new camera matrix (P) based on the desired undistortion
        P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, size, R, balance=1)
        
        m1type = cv2.CV_32FC1
        
        # Generate the undistortion and rectification transformation map
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, R, P, size, m1type)
        
        # Apply the undistortion transformation
        undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)

        # Store the undistorted image and the original distorted image
        self.bw_image_T265 = undistorted_image
        self.bw_image_distorted = image

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
        misalignment = 3.5 #degrees between T265 and D435
        pitch = (-m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi) + misalignment
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
            if x > self.realsense_color_W:
                x = self.realsense_color_W
            if x < 0:
                x = 0
            if y > self.realsense_color_H:
                y = self.realsense_color_H
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

    def fish_pixel_to_depth_pixel(self, x, y):
        print("hello")
        depthx, depthy = rs.rs2_project_color_pixel_to_depth_pixel(
                    self.depth_frame.get_data(), 
                    self.depth_scale, 
                    self.depth_min, 
                    self.depth_max, 
                    self.intrinsics_depth, 
                    self.intrinsics_fish, 
                    self.depth_to_color_extrinsics, 
                    self.color_to_depth_extrinsics, 
                    [x,y])
        print(depthx, depthy)
        return depthx, depthy
    

rsm = RealSenseManager()


#show the depth image



while True:
    rsm.update_realsense_data()
    frame = rsm.depth_frame
    depth_image = np.asanyarray(frame.get_data())
    depth_colormap = rsm.color_image_D435 #cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    grey_image = rsm.bw_image_T265

    grey_image_bgr = cv2.cvtColor(grey_image, cv2.COLOR_GRAY2BGR)

    # Resize the depth colormap to new dimensions (e.g., half the size of the original)
    new_width = int(depth_colormap.shape[1] / 2.5)
    new_height = int(depth_colormap.shape[0] / 2.5)
    resized_depth_colormap = cv2.resize(depth_colormap, (new_width, new_height), interpolation=cv2.INTER_AREA)

    # Define the top-left corner of the ROI on the grayscale image
    top_left_x = 150  # Change this to move the overlay position
    top_left_y = 300  # Change this to move the overlay position

    # Overlay the resized depth colormap onto the grayscale image
    grey_image_bgr[top_left_y:top_left_y+new_height, top_left_x:top_left_x+new_width] = resized_depth_colormap




    cv2.imshow('Depth Image', grey_image_bgr)
    time.sleep(0.1)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break
# sensing_advanced.py
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

DEBUG_MODE = False  # Set to True to see confidence and occupancy maps

# world coordinate system
# up +Y
# /\    _
# |     /| forward +Z
# |    /
# |   /
# |  /
# | /
# |/______> right +X


class TimingLogger:
    def __init__(self):
        self.timings = {}
        self.start_times = {}

    def start(self, operation):
        self.start_times[operation] = time.time()

    def end(self, operation):
        if operation in self.start_times:
            duration = time.time() - self.start_times[operation]
            if operation not in self.timings:
                self.timings[operation] = []
            self.timings[operation].append(duration)
            del self.start_times[operation]

    def print_stats(self):
        print("\nTiming Statistics:")
        for op, times in self.timings.items():
            avg = sum(times) / len(times)
            max_t = max(times)
            print(f"{op:25s} Avg: {avg*1000:6.1f}ms  Max: {max_t*1000:6.1f}ms")
        self.timings.clear()

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

    def map_image_to_redis(self, name: str, array: np.ndarray) -> None:
        """
        Send raw map data to Redis with clear naming convention
        
        Prefix conventions:
        raw_     - Raw sensor data (e.g., raw_depth_map)
        overlay_ - Visualization overlays (handled by navigation/showmap)
        """
        h, w = array.shape[:2]
        shape = struct.pack('>II', h, w)
        encoded = shape + array.tobytes()
        self.r.set(name, encoded)

class RealSenseManager:
    def __init__(self):
        self.realsense_depth_W = 424  # Lowest supported depth resolution
        self.realsense_depth_H = 240
        self.realsense_color_W = 640  # Keep higher resolution for tag detection
        self.realsense_color_H = 360

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
        self.configD435.enable_stream(rs.stream.depth, self.realsense_depth_W, self.realsense_depth_H, rs.format.z16, 15)  # High fps at low res
        self.configD435.enable_stream(rs.stream.color, self.realsense_color_W, self.realsense_color_H, rs.format.bgr8, 15)  # Keep color at 15fps


        self.pipelineD435.start(self.configD435)

        self.profileD435 = self.pipelineD435.get_active_profile()
        # Get depth sensor for optional configuration
        depth_sensor = self.profileD435.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, 360)

        self.depth_scale = depth_sensor.get_depth_scale()
        self.depth_min = 0.2  # meter
        self.depth_max = 6.0  # meter

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

        self.camera_height = 0.23  # mounting height of the depth camera vs. ground

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
        self.depth_image_D435 = np.asanyarray(self.depth_frame.get_data())
    
    def get_bw_image_T265(self):
        fisheye_frame = self.framesT265.get_fisheye_frame(1)  # Get the fisheye frame from the first camera
        image = np.asanyarray(fisheye_frame.get_data())

        K = np.array([[self.intrinsics_fish.fx, 0, self.intrinsics_fish.ppx],
                      [0, self.intrinsics_fish.fy, self.intrinsics_fish.ppy],
                      [0, 0, 1]])
        D = np.array(self.intrinsics_fish.coeffs[:4])

        R = np.eye(3)  # Assuming no rotation (identity matrix)
        size = (self.intrinsics_fish.width, self.intrinsics_fish.height)
        m1type = cv2.CV_32FC1
        P = K  # Assuming the new camera matrix is the same as K for simplicity

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, R, P, size, m1type)
        undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)
        self.bw_image_T265 = undistorted_image

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
        misalignment = 1 #degrees between T265 and D435
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
            # Scale coordinates from color to depth resolution
            scale_x = self.realsense_depth_W / self.realsense_color_W
            scale_y = self.realsense_depth_H / self.realsense_color_H
            
            # Convert color coordinates to relative position (0-1)
            rel_x = x / self.realsense_color_W
            rel_y = y / self.realsense_color_H
            
            # Convert to depth coordinates
            depth_x = int(rel_x * self.realsense_depth_W)
            depth_y = int(rel_y * self.realsense_depth_H)
            
            return depth_x, depth_y
        else:  # T265
            print("Converting T265 pixel to depth pixel")
            # Updated T265 to D435 transformation with improved scaling
            # These values need to be calibrated for your specific setup
            t265_to_d435 = np.array([
                [1, 0, 0, 0.009],  # x offset in meters
                [0, 1, 0, 0.021],  # y offset in meters
                [0, 0, 1, 0.027],  # z offset in meters
                [0, 0, 0, 1]
            ])
            
            point_t265 = self._t265_pixel_to_3d(x, y)
            if point_t265 is None:
                print("Failed to get 3D point from T265")
                return None, None
            
            # Convert to homogeneous coordinates
            point_homogeneous = np.append(point_t265, 1)
            
            # Apply transformation
            point_d435 = t265_to_d435 @ point_homogeneous
            print(f"Point in D435 coordinates: {point_d435[:3]}")
            
            # Project to D435 image plane
            depth_x, depth_y = rs.rs2_project_point_to_pixel(
                self.intrinsics_depth,
                point_d435[:3]
            )
            print(f"Final depth pixels: x={depth_x}, y={depth_y}")
            
            return depth_x, depth_y
        
    def _t265_pixel_to_3d(self, x, y):
        """Convert T265 pixel coordinates to 3D point with hybrid distance estimation"""
        #print(f"T265 input pixel: x={x}, y={y}")
        
        # First normalize coordinates to get direction
        x_normalized = (x - self.intrinsics_fish.ppx) / self.intrinsics_fish.fx
        y_normalized = (y - self.intrinsics_fish.ppy) / self.intrinsics_fish.fy
        
        # Calculate ray direction with inverted y coordinate
        ray_direction = np.array([x_normalized, -y_normalized, 1.0])
        ray_direction = ray_direction / np.linalg.norm(ray_direction)
        
        # Scale coordinates to D435's FOV to check if tag might be visible to it
        d435_relative_x = x * (self.realsense_depth_W / self.intrinsics_fish.width)
        
        # Check if the point would be within D435's horizontal FOV
        if 0 <= d435_relative_x < self.realsense_depth_W:
            # Use D435's depth data from the top half of the frame at this horizontal position
            depth_samples = []
            # Sample several rows in the top half of the depth frame
            for y_sample in range(self.realsense_depth_H // 4, self.realsense_depth_H // 2):
                depth = self.depth_frame.get_distance(int(d435_relative_x), y_sample)
                if depth > 0:  # Valid depth reading
                    depth_samples.append(depth)
            
            if depth_samples:  # If we got any valid depth readings
                # Use median depth to avoid outliers
                assumed_distance = np.median(depth_samples)
                #print(f"Using D435 measured depth: {assumed_distance}m")
            else:
                # Fallback to assumption if no valid depth readings
                assumed_distance = 4.0
                #print(f"No valid D435 depth, using assumed distance: {assumed_distance}m")
        else:
            # Point is outside D435 FOV, use assumed distance
            assumed_distance = 4.0
            #print(f"Point outside D435 FOV, using assumed distance: {assumed_distance}m")
        
        # Calculate final 3D point
        point = ray_direction * assumed_distance
        #print(f"Ray direction: {ray_direction}")
        #print(f"Final 3D point: {point}")
        
        return point

    def _d435_color_to_depth(self, x, y, intrinsics):
        """Original D435 color to depth conversion"""
        depthx, depthy = rs.rs2_project_color_pixel_to_depth_pixel(
            self.depth_frame.get_data(), 
            self.depth_scale, 
            self.depth_min, 
            self.depth_max, 
            self.intrinsics_depth, 
            intrinsics, 
            self.depth_to_color_extrinsics, 
            self.color_to_depth_extrinsics, 
            [x,y]
        )

        if depthx > self.realsense_depth_W:
            depthx = self.realsense_depth_W - 1
        if depthy > self.realsense_depth_H:
            depthy = self.realsense_depth_H - 1
        
        return depthx, depthy 
    
class AprilTagDetector:
    def __init__(self, realsensemanager, redismanager):
        # Configure detector with lower detection threshold
        detector_config = apriltag.DetectorOptions(
            families='tag36h11',     # Standard tag family
            border=1,                # Border size (in pixels)
            nthreads=4,             # Use multiple threads for detection
            quad_decimate=1.0,      # Don't decimate (reduce) image resolution
            quad_blur=0.0,          # No blurring
            refine_edges=True,      # Spend more time trying to find edges
            refine_decode=True,     # Spend more time trying to decode tags
            refine_pose=True,       # Refine pose estimates
            debug=False,            # Disable debug output
            quad_contours=True      # Use quad contours
        )
        self.detector = apriltag.Detector(detector_config)
        
        self.rsm = realsensemanager
        self.rdm = redismanager

        self.target_world_coords = None
        self.target_car_coords = None

    def detect_tags(self):
        detected = False
        
        # Try D435 first
        tags = self.detector.detect(self.rsm.gray_image_D435)
        for tag in tags:
            try:
                x, y = tag.center
                x, y = self.rsm.color_pixel_to_depth_pixel(x, y, "D435")
                if 0 <= x < self.rsm.realsense_depth_W and 0 <= y < self.rsm.realsense_depth_H:
                    self.rdm.set_data('log_detect_cam', "D435", 3000)
                    detected = True
                    
                    # Get 3D coordinates using D435's depth
                    cx, cy, cz = self.rsm.pixel_to_car_coord(int(x), int(y))
                    cwx, cwy, cwz = self.rsm.car_coord_to_world_coord(cx, cy, cz)
                    self.target_world_coords = [cwx, cwy, cwz]
                    
                    # Convert back to car coordinates for navigation
                    target_car_x, target_car_y, target_car_z = self.rsm.world_coord_to_car_coord(cwx, cwy, cwz)
                    target_coords_bytes = struct.pack('%sf' % 3, target_car_x, target_car_y, target_car_z)
                    self.rdm.set_data('target_car_coords', target_coords_bytes, 3000)

                    break
                    
            except Exception as e:
                print(f"Error processing D435 tag: {e}")
                continue
        
        # If no detection on D435, try T265
        if not detected:
            tags = self.detector.detect(self.rsm.bw_image_T265)
            for tag in tags:
                try:
                    x, y = tag.center
                    point_3d = self.rsm._t265_pixel_to_3d(x, y)
                    
                    if point_3d is not None:
                        cx, cy, cz = point_3d
                        cwx, cwy, cwz = self.rsm.car_coord_to_world_coord(cx, cy, cz)
                        self.target_world_coords = [cwx, cwy, cwz]
                        
                        target_car_x, target_car_y, target_car_z = self.rsm.world_coord_to_car_coord(cwx, cwy, cwz)
                        target_coords_bytes = struct.pack('%sf' % 3, target_car_x, target_car_y, target_car_z)
                        self.rdm.set_data('target_car_coords', target_coords_bytes, 3000)
                        self.rdm.set_data('log_detect_cam', "T265", 3000)
                        detected = True

                        break
                        
                except Exception as e:
                    print(f"Error processing T265 tag: {e}")
                    continue
        
        # Clear target if no detection
        if not detected:
            self.target_world_coords = None

class MapManager:
    def __init__(self, width, height, base_height, realsensemanager, redismanager):
        # Change map size but maintain physical dimensions
        self.cm_per_pixel = 2  # Now 2cm per pixel instead of 1
        self.mapW = width // self.cm_per_pixel  # 400 instead of 800
        self.mapH = height // self.cm_per_pixel  # 400 instead of 800
        self.map_base_height = base_height
        
        # Three-layer map setup with smaller dimensions
        self.map = np.zeros((self.mapW, self.mapH, 3), dtype=np.float32)
        self.map[:, :, 0] = self.map_base_height
        self.map[:, :, 1] = 0.5
        self.map[:, :, 2] = 0.0
        
        self.last_update_times = np.full((self.mapW, self.mapH), time.time())
        self.realsensemanager = realsensemanager
        self.redismanager = redismanager
        
        # Update car position for new resolution
        self.car_position_on_map = 250 // self.cm_per_pixel  # Scale car position too
        
        # Create new decay mask for smaller map
        self._create_decay_region_mask()
        

    def _create_decay_region_mask(self):
        """Creates decay mask scaled for new resolution"""
        mask = np.zeros((self.mapW, self.mapH), dtype=np.uint8)
        car_x = self.mapW // 2
        car_y = self.mapH - self.car_position_on_map
        
        # Use 90 degree FOV to match showmap
        camera_fov = 93.0
        fov_rad = np.radians(camera_fov)
        
        # Calculate cone width at top of map
        distance_to_top = car_y  # Distance from camera to top of map
        cone_width = int(2 * distance_to_top * np.tan(fov_rad / 2))
        
        # Define decay region points
        decay_region = np.array([
            [car_x + 13//self.cm_per_pixel, car_y],  # Right corner of car front
            [car_x - 13//self.cm_per_pixel, car_y],  # Left corner of car front
            [car_x - cone_width//2, 0],              # Left edge at top of map
            [car_x + cone_width//2, 0]               # Right edge at top of map
        ], np.int32)
        
        decay_region = decay_region.reshape((-1, 1, 2))
        cv2.fillPoly(mask, [decay_region], 1)
        self.decay_region_mask = mask

    def bayesian_update_vectorized(self, map_x, map_y, measurement_heights, measurement_confidences):
        current_time = time.time()
        
        # Create time since update array
        time_since_update = current_time - self.last_update_times[map_y, map_x]
        
        # Decay confidence
        current_confidences = self.map[map_y, map_x, 2] * np.exp(-self.temporal_decay_rate * time_since_update)
        
        # Get priors
        priors = self.map[map_y, map_x, 1]
        
        # Calculate alpha for height updates
        alphas = np.zeros_like(current_confidences)
        mask = current_confidences > 0
        alphas[mask] = measurement_confidences[mask] / (measurement_confidences[mask] + current_confidences[mask])
        
        # Update heights
        old_heights = self.map[map_y, map_x, 0]
        new_heights = np.where(mask,
                            (1 - alphas) * old_heights + alphas * measurement_heights,
                            measurement_heights)
        
        # Calculate likelihoods
        likelihoods = np.where(measurement_heights > self.map_base_height,
                            self.sensor_noise_model,
                            1 - self.sensor_noise_model)
        
        # Bayesian update for occupancy
        posteriors = (likelihoods * priors) / (likelihoods * priors + (1 - likelihoods) * (1 - priors))
        
        # Calculate new confidences
        new_confidences = np.minimum(current_confidences + measurement_confidences, self.max_confidence)
        
        # Update map
        self.map[map_y, map_x, 0] = new_heights
        self.map[map_y, map_x, 1] = posteriors
        self.map[map_y, map_x, 2] = new_confidences
        self.last_update_times[map_y, map_x] = current_time

    def update_map_and_obstacles_vectorized(self, depth_frame, max_climb_height, rotation, color_image_D435):
        """Optimized map update with reduced resolution"""
        h_start = int(self.realsensemanager.realsense_depth_H / 4)
        depth_data = np.asanyarray(depth_frame.get_data())[h_start:, :]
        
        # Create coordinate grids
        h, w = depth_data.shape
        y_coords, x_coords = np.mgrid[h_start:self.realsensemanager.realsense_depth_H, 
                                    0:self.realsensemanager.realsense_depth_W].astype(np.float32)
        
        # Deproject to 3D
        Z = depth_data * self.realsensemanager.depth_scale
        X = (x_coords - self.realsensemanager.intrinsics_depth.ppx) * Z / self.realsensemanager.intrinsics_depth.fx
        Y = (y_coords - self.realsensemanager.intrinsics_depth.ppy) * Z / self.realsensemanager.intrinsics_depth.fy
        
        points = np.stack((X, Y, Z), axis=-1)
        points_rotated = np.einsum('ij,klj->kli', rotation, points)
        points_rotated[:, :, 1] = self.realsensemanager.camera_height - points_rotated[:, :, 1]
        
        # Scale coordinates for new resolution
        map_x = (points_rotated[:, :, 0] * 100/self.cm_per_pixel + (self.mapW / 2) - 3).astype(np.int32)
        map_y = (self.mapH - points_rotated[:, :, 2] * 100/self.cm_per_pixel - self.car_position_on_map).astype(np.int32)
        height_values = 100 + points_rotated[:, :, 1] * 100
        
        valid_mask = (
            (points_rotated[:, :, 1] <= self.ignore_above_height) &
            (map_x >= 0) & (map_x < self.mapW) &
            (map_y >= 0) & (map_y < self.mapH) &
            (Z > 0)
        )
        
        valid_x = map_x[valid_mask]
        valid_y = map_y[valid_mask]
        valid_heights = height_values[valid_mask]
        valid_confidences = np.maximum(0.1, 1.0 - (Z[valid_mask] / 5.0))
        
        self.bayesian_update_vectorized(valid_x, valid_y, valid_heights, valid_confidences)

    def update_variables(self):
        self.target_memory_time = int(self.redismanager.get_float('target_memory_time', 1000))
        self.tap_target_memory_time = int(self.redismanager.get_float('tap_target_memory_time', 1000))
        self.map_base_height = int(self.redismanager.get_float('map_base_height', 100))
        self.map_clear_all_interval = self.redismanager.get_float('map_clear_all_interval', 3)
        self.map_clear_visible_cone_interval = self.redismanager.get_float('map_clear_visible_cone_interval', 0.5)
        self.depth_pixel_horizontal_raster = int(self.redismanager.get_float('depth_pixel_horizontal_raster', 5))
        self.depth_pixel_vertical_raster = int(self.redismanager.get_float('depth_pixel_vertical_raster', 1))
        self.ignore_above_height = self.redismanager.get_float('ignore_above_height', 0.4)
        self.square_range = int(self.redismanager.get_float('square_range', 6))
        self.draw_grid = int(self.redismanager.get_float('draw_grid', 1))
        self.draw_obstacles = int(self.redismanager.get_float('draw_obstacles', 1))
        self.max_climb_height = self.redismanager.get_float('max_climb_height', 10)
        
        # Add probabilistic mapping parameters
        self.temporal_decay_rate = self.redismanager.get_float('temporal_decay_rate', 0.1)
        self.confidence_threshold = self.redismanager.get_float('confidence_threshold', 0.7)
        self.occupancy_threshold = self.redismanager.get_float('occupancy_threshold', 0.7)
        self.sensor_noise_model = self.redismanager.get_float('sensor_noise_model', 0.9)
        self.max_confidence = self.redismanager.get_float('max_confidence', 10.0)


    def rotate_and_move_map(self, angle, up, right):
        """Optimized single-pass rotation and translation"""
        if abs(angle) < 0.01 and abs(up) < 0.01 and abs(right) < 0.01:
            return  # Skip if movement is negligible
            
        # Combine rotation and translation into single matrix
        center = (self.mapW // 2, self.mapH - self.car_position_on_map)
        rot_mat = cv2.getRotationMatrix2D(center, angle, 1.0).astype(np.float32)
        
        # Add translation directly to rotation matrix
        rot_mat[0, 2] += right
        rot_mat[1, 2] += up
        
        # Single warpAffine for all channels
        borderValue = np.array([self.map_base_height, 0.5, 0])  # Default values for each channel
        self.map = cv2.warpAffine(
            self.map,
            rot_mat,
            (self.mapW, self.mapH),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=borderValue
        )

    def decay_unobserved_areas(self):
        """Fully vectorized decay calculation"""
        current_time = time.time()
        
        # Calculate all decays in one operation
        # Only calculate within the decay mask to save computation
        decay_mask = self.decay_region_mask == 1
        if not np.any(decay_mask):
            return
            
        # Calculate decay factors directly for masked area
        time_diff = current_time - self.last_update_times[decay_mask]
        decay_factor = np.exp(-self.temporal_decay_rate * time_diff)
        
        # Apply decay to confidence values
        self.map[decay_mask, 2] *= decay_factor
        
        # Fast update of low confidence areas using boolean indexing
        low_conf = decay_mask & (self.map[:, :, 2] < 0.2)
        if np.any(low_conf):
            self.map[low_conf, 1] = 0.5  # Reset occupancy
            self.map[low_conf, 0] = self.map_base_height  # Reset height

    def publish_raw_maps(self, redis_manager):
        """
        Publish combined height and confidence map to Redis
        Height: 12 bits (-2048 to +2047 relative to base height)
        Confidence: 4 bits (0-15 levels)
        """
        # Get height values relative to base height
        height_relative = self.map[:, :, 0] - self.map_base_height
        
        # Scale confidence from 0-1 to 0-15
        confidence_scaled = (self.map[:, :, 2] * 15).astype(np.uint16)
        # Clip to ensure we stay within 4 bits
        confidence_scaled = np.clip(confidence_scaled, 0, 15)
        
        # Combine height and confidence:
        # Shift height left by 4 bits and add confidence
        combined_map = ((height_relative.astype(np.uint16) & 0x0FFF) << 4) | confidence_scaled
        
        # Send combined map to Redis
        redis_manager.map_image_to_redis('raw_map', combined_map.astype(np.uint16))


rsm = RealSenseManager()
rdm = RedisManager()
atd = AprilTagDetector(rsm, rdm)
mapc = MapManager(
    width=800,   # Keep physical width the same
    height=800,  # Keep physical height the same
    base_height=100,
    realsensemanager=rsm,
    redismanager=rdm
)
# pd = PersonDetector(rsm)
app_start_time = time.time()
rsm.update_realsense_data()
timer = TimingLogger()
loop_counter = 0 
while True:
    timer.start('full_loop')
    loop_start_time = time.time()
    
    timer.start('realsense_update')
    rsm.update_realsense_data()
    timer.end('realsense_update')

    timer.start('map_update')
    mapc.update_variables()
    rsm.get_rotation()
    mapc.update_map_and_obstacles_vectorized(
        depth_frame=rsm.depth_frame,
        max_climb_height=10,
        rotation=rsm.rotation,
        color_image_D435=rsm.color_image_D435
    )
    timer.end('map_update')

    timer.start('apriltag')
    atd.detect_tags()
    timer.end('apriltag')

    timer.start('map_maintenance')

    timer.start('decay')
    mapc.decay_unobserved_areas()
    timer.end('decay')

    timer.start('get_yaw')
    rsm.get_yaw_increment()
    timer.end('get_yaw')

    timer.start('rotate_move')
    mapc.rotate_and_move_map(
        rsm.yaw_increment, 
        rsm.car_in_world_coord_z_increment * 100, 
        -rsm.car_in_world_coord_x_increment * 100
    )
    timer.end('rotate_move')

    timer.end('map_maintenance')
    
    timer.start('redis_updates')
    mapc.publish_raw_maps(rdm)
    rdm.set_data('log_uptime', time.time() - app_start_time, 1000)
    rdm.set_data('log_sensing_running', 'on', 1000)
    rdm.set_data('current_speed', rsm.speed)
    rdm.map_image_to_redis('D435_image', rsm.color_image_D435)
    #rdm.map_image_to_redis('D435_depth_image', rsm.depth_image_D435)
    #rdm.map_image_to_redis('T265_image', rsm.bw_image_T265)
    timer.end('redis_updates')

    # Log timing
    timer.end('full_loop')
    sensing_time = time.time() - loop_start_time
    rdm.set_data('log_sensing_time', sensing_time, 1000)

    # Optional: Add debug visualizations
    if DEBUG_MODE:
        # Send confidence map for visualization
        confidence_map = (mapc.map[:, :, 2] * 255).astype(np.uint8)
        rdm.map_image_to_redis('confidence_map', confidence_map)
        
        # Send occupancy map for visualization
        occupancy_map = (mapc.map[:, :, 1] * 255).astype(np.uint8)
        rdm.map_image_to_redis('occupancy_map', occupancy_map)
    
    loop_counter += 1
    if loop_counter % 50 == 0:
        timer.print_stats()

    # print(mapc.depth_pixel_horizontal_raster, mapc.depth_pixel_vertical_raster)
# %% Performance
import csv
import os
import time
import logging

# %%
import cupy as cp
import cupyx as cpx
import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo

# %%
ZED_DEPTH_TOPIC = '/zed2i/zed_node/depth/depth_registered'  # '/islam/zed/depth'  # '/islam/zed_depth'
ZED_CAMERA_INFO_TOPIC = '/zed2i/zed_node/depth/camera_info'  # '/islam/zed/camera_info'
ZED_RGB_TOPIC = '/zed2i/zed_node/left/image_rect_color'  # '/islam/zed/rgb'

VLP_TOPIC = "/velodyne_points"

LOAM_ODOM_TOPIC = '/islam/loam_odom'

PG_DEPTH_TOPIC = "/islam/pg_depth"
PG_CAMERA_INFO_TOPIC = '/islam/pg_camera_info'
PG_RGB_TOPIC = '/islam/pg_rgb'
PG_ODOM_TOPIC = '/islam/pg_odom'
PG_FUSED_PC_TOPIC = "/islam/pg_fused_pointcloud"
ZED_PC_TOPIC = "/islam/zed_pointcloud"

VLP_FILTERED_PC_TOPIC = "/islam/vlp_filtered_pointcloud"

# %%
ZED_V = 376
ZED_H = 672
ZED_H_ANGLE = 87
ZED_V_ANGLE = 56

LiDAR_V = 16
LiDAR_ANGLE = 30.2  # 0.2 degree precaution


# %%
def sph_to_cart_pts(pts):
    pts[:, 1] = cp.radians(pts[:, 1])
    pts[:, 2] = cp.radians(pts[:, 2])

    # Convert spherical coordinates to Cartesian coordinates
    x = pts[:, 0] * cp.cos(pts[:, 1]) * cp.cos(pts[:, 2])
    y = pts[:, 0] * cp.cos(pts[:, 1]) * cp.sin(pts[:, 2])
    z = pts[:, 0] * cp.sin(pts[:, 1])

    return cp.asarray([x, y, z]).T


def cart_to_sph_pts(pts):
    # Convert to CuPy array
    pts = cp.asarray(pts)

    # Convert to spherical coordinates
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = cp.sqrt(x ** 2 + y ** 2 + z ** 2)
    theta = cp.arctan(z / cp.sqrt(x ** 2 + y ** 2))
    phi = cp.arctan(y / x)

    return cp.column_stack((r, cp.degrees(theta), cp.degrees(phi)))
    # return appendSpherical_np(pts)[:,3:6]


# %%
def msg2pts(msg):
    return cp.array(list(pc2.read_points(msg, field_names=("x", "y", "z"))))


# %%
def depth_to_sph_pts(depth):
    # Make sure input depth array is a CuPy array
    depth = cp.array(depth)

    # get the shape of the input array
    m, n = depth.shape
    azimuth_const = ZED_H_ANGLE / n
    polar_const = ZED_V_ANGLE / m

    # Create a grid of row and col indices
    row_indices, col_indices = cp.meshgrid(cp.arange(m), cp.arange(n), indexing='ij')

    # Calculate polar and azimuth angles
    polar_angles = row_indices * polar_const
    azimuth_angles = col_indices * azimuth_const

    # Stack the depth, polar_angles, and azimuth_angles along the last dimension
    pts = cp.stack((depth, polar_angles, azimuth_angles), axis=-1)

    # Reshape the pts array to the desired output shape (m * n, 3)
    pts = pts.reshape(m * n, 3)

    return pts


# %%
def lpf(img, ncutoff):
    # Apply 2D FFT to the image
    f = cp.fft.fft2(img)

    # Shift the zero frequency component to the center of the spectrum
    fshift = cp.fft.fftshift(f)

    # Create a circular mask of the same size as the spectrum
    rows, cols = img.shape
    crow, ccol = rows // 2, cols // 2
    mask = np.zeros((rows, cols), np.uint8)
    cutoff = int(min(crow, ccol) * ncutoff)
    cv2.circle(mask, (ccol, crow), cutoff, 1, -1)
    # cv2.ellipse(mask, (ccol, crow), (1, 2) * cutoff, 0, 0, 360,  1, -1)

    mask = cp.asarray(mask)

    # Apply the mask to the shifted spectrum
    fshift_filtered = fshift * mask

    # Shift the zero frequency component back to the corner of the spectrum
    f_filtered = cp.fft.ifftshift(fshift_filtered)

    # Apply the inverse 2D FFT to the filtered spectrum
    img_filtered = cp.fft.ifft2(f_filtered)
    img_filtered = cp.real(img_filtered)

    return img_filtered


def pg(zed_depth, vlp_depth, ncutoff, threshold=100):
    ncutoff = ncutoff / 10

    mask = vlp_depth > 0
    filtered = zed_depth
    filtered[mask] = vlp_depth[mask]

    while threshold > 0:
        filtered = lpf(filtered, ncutoff)
        filtered[mask] = vlp_depth[mask]

        threshold -= 1

    return filtered


# %%
def remap(old_value, old_min, old_max, new_min, new_max):
    # Function to map a value from one range to another
    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min) * new_range) / old_range) + new_min
    return new_value


# %%
rospy.init_node('sf', anonymous=True)
rospy.set_param('/rosgraph/log_level', logging.DEBUG)

# Global variables for performance logging
processing_times = []
frame_counter = 0
last_report_time = time.time()
report_interval_frames = 100  # Report every 100 frames
report_interval_seconds = 5  # Report every 5 seconds

# --- ALGORITHM PARAMETERS TO BE LOGGED ---
CURRENT_NCUTOFF = 10  # Example value, you will change this
CURRENT_THRESHOLD = 1 # Example value, you will change this
# You can add more parameters here if needed, e.g., LiDAR filtering range, etc.
# ----------------------------------------

# CSV logging setup
log_dir = os.path.expanduser("./logs")  # Log to user's home directory
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(log_dir, f"fusion_performance_{time.strftime('%Y%m%d_%H%M%S')}.csv")
log_file = open(log_filename, 'w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow([
    'Timestamp',
    'FrameNumber',
    'FusionTime_ms',
    'TotalCallbackTime_ms',
    'ZED_Timestamp_sec',
    'ncutoff',  # New column
    'threshold'  # New column
])  # CSV Header


# Register a shutdown hook to close the log file cleanly
def shutdown_hook():
    rospy.loginfo("Shutting down sensor fusion node. Closing log file.")
    if log_file:
        log_file.close()


rospy.on_shutdown(shutdown_hook)

# %%
pg_depth_p = rospy.Publisher(PG_DEPTH_TOPIC, Image, queue_size=50)
pg_camera_info_p = rospy.Publisher(PG_CAMERA_INFO_TOPIC, CameraInfo, queue_size=50)
pg_rgb_p = rospy.Publisher(PG_RGB_TOPIC, Image, queue_size=50)
pg_odom_p = rospy.Publisher(PG_ODOM_TOPIC, Odometry, queue_size=10)
pg_fused_pc_p = rospy.Publisher(PG_FUSED_PC_TOPIC, PointCloud2, queue_size=10)
zed_pc_p = rospy.Publisher(ZED_PC_TOPIC, PointCloud2, queue_size=10)

vlp_filtered_pc_p = rospy.Publisher(VLP_FILTERED_PC_TOPIC, PointCloud2, queue_size=10)

pg_camera_info_msg = CameraInfo()
pg_rgb_msg = Image()
pg_odom_msg = Odometry()
bridge = CvBridge()

zed_img_init = rospy.wait_for_message(ZED_DEPTH_TOPIC, Image)
ZED_V, ZED_H = cp.array(bridge.imgmsg_to_cv2(zed_img_init, "32FC1")).shape
# --- Store ZED's initial frame_id ---
zed_depth_frame_id = zed_img_init.header.frame_id
rospy.loginfo(f"Detected ZED Depth Frame ID: {zed_depth_frame_id}")
# ------------------------------------

vlp_depth = cp.zeros((ZED_V, ZED_H), dtype=cp.float32)
vlp_mean = 0

pg_img = None


# %%
def zed_callback(zed_img: Image):
    global vlp_depth, vlp_mean, processing_times, frame_counter, last_report_time, csv_writer, log_file, zed_depth_frame_id
    global CURRENT_NCUTOFF, CURRENT_THRESHOLD # Access global parameters

    rospy.loginfo("zed callback")

    # Record start time for the entire zed_callback processing
    total_start_time = time.time()
    zed_msg_timestamp = zed_img.header.stamp.to_sec() # Get ROS timestamp

    # ZED Preproc
    zed_depth = cp.array(bridge.imgmsg_to_cv2(zed_img, "32FC1"))
    zed_depth[cp.isnan(zed_depth)] = vlp_mean
    zed_depth[zed_depth > 20] = vlp_mean

    # Sensor Fusion
    fusion_start_time = time.time() # Start timing only the fusion part
    pg_depth = pg(zed_depth.copy(), vlp_depth.copy(), ncutoff=CURRENT_NCUTOFF, threshold=CURRENT_THRESHOLD)
    fusion_end_time = time.time() # End timing fusion part
    fusion_time_ms = (fusion_end_time - fusion_start_time) * 1000

    processing_times.append(fusion_time_ms)
    frame_counter += 1

    # Publish Image
    global pg_img
    pg_img = pg_depth
    pg_depth_msg = bridge.cv2_to_imgmsg(pg_depth.get())
    pg_depth_msg.header.stamp = zed_img.header.stamp
    pg_depth_p.publish(pg_depth_msg)
    rospy.logwarn("pg_depth published")

    # --- NEW: Convert Fused Depth to PointCloud2 and Publish ---
    # Need camera intrinsics for this. Assume pg_camera_info_msg is already populated.
    # Convert depth image to Cartesian points first
    # This part can be computationally heavy, consider if you need it for every frame.
    # If not, you might publish it less frequently or only for debugging.

    # Ensure pg_camera_info_msg has valid data before proceeding
    if pg_camera_info_msg.K and pg_camera_info_msg.P:
        def depth_to_cart_pts(depth, camera_info_msg):
            # Get camera intrinsics from CameraInfo message
            fx = camera_info_msg.K[0]
            fy = camera_info_msg.K[4]
            cx = camera_info_msg.K[2]
            cy = camera_info_msg.K[5]

            # Convert depth image to point cloud
            rows, cols = depth.shape
            u, v = cp.meshgrid(cp.arange(cols), cp.arange(rows))

            # CuPy operations for point cloud generation
            Z = depth
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            # Stack into N x 3 points
            cart_pts = cp.stack((X, Y, Z), axis=-1).reshape(-1, 3)

            # Filter out invalid points (e.g., where Z is 0 or NaN)
            valid_mask = (Z.flatten() > 0) & cp.isfinite(Z.flatten())
            cart_pts = cart_pts[valid_mask]
            return cart_pts

        fused_cart_pts = depth_to_cart_pts(pg_depth, camera_info_msg=pg_camera_info_msg)
        zed_cart_pts = depth_to_cart_pts(zed_depth, camera_info_msg=pg_camera_info_msg)

        def cart_pts_to_pc_msg(cart_pts, header):
            # Convert CuPy array to NumPy for pc2.create_cloud
            cart_pts_np = cart_pts.get()

            fields = [
                pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            ]
            pc_msg = pc2.create_cloud(header, fields, cart_pts_np)
            return pc_msg

        # Create PointCloud2 message
        header = zed_img.header
        header.frame_id = zed_depth_frame_id  # pg_camera_info_msg.header.frame_id # Ensure frame_id is correct (usually camera_link/optical_frame)

        pg_fused_pc_msg = cart_pts_to_pc_msg(fused_cart_pts, header)
        zed_pc_msg = cart_pts_to_pc_msg(zed_cart_pts, header)

        pg_fused_pc_p.publish(pg_fused_pc_msg)
        zed_pc_p.publish(zed_pc_msg)
        rospy.loginfo("publish_pg")
    else:
        rospy.logwarn("CameraInfo not received yet or invalid, skipping fused point cloud publication.")
        rospy.logwarn_throttle(5, "CameraInfo not received yet or invalid, skipping fused point cloud publication.")
    # --- END NEW ---

    # Publish aux info
    pg_rgb_msg.header.stamp = zed_img.header.stamp
    pg_camera_info_msg.header.stamp = zed_img.header.stamp
    pg_odom_msg.header.stamp = zed_img.header.stamp
    pg_rgb_p.publish(pg_rgb_msg)
    pg_camera_info_p.publish(pg_camera_info_msg)
    pg_odom_p.publish(pg_odom_msg)
    rospy.loginfo("pg_odom published")

    # End timing for the entire zed_callback processing
    total_end_time = time.time()
    total_processing_time_ms = (total_end_time - total_start_time) * 1000

    # Log data to CSV
    csv_writer.writerow([
        time.time(),  # System timestamp
        frame_counter,
        fusion_time_ms,
        total_processing_time_ms,
        zed_msg_timestamp,  # Original ZED message timestamp
        CURRENT_NCUTOFF,  # Log ncutoff
        CURRENT_THRESHOLD  # Log threshold
    ])
    log_file.flush()  # Ensure data is written to disk immediately

    # Report performance metrics periodically
    current_time = time.time()
    if frame_counter >= report_interval_frames or (current_time - last_report_time) >= report_interval_seconds:
        if processing_times: # Ensure there's data to calculate
            avg_fusion_time = sum(processing_times) / len(processing_times)
            max_fusion_time = max(processing_times)
            min_fusion_time = min(processing_times)
            rospy.loginfo(f"--- Performance Report (Last {len(processing_times)} frames) ---")
            rospy.loginfo(f"  Parameters: ncutoff={CURRENT_NCUTOFF}, threshold={CURRENT_THRESHOLD}") # Log params in console report
            rospy.loginfo(f"  Avg Fusion Time: {avg_fusion_time:.2f} ms")
            rospy.loginfo(f"  Max Fusion Time: {max_fusion_time:.2f} ms")
            rospy.loginfo(f"  Min Fusion Time: {min_fusion_time:.2f} ms")
            rospy.loginfo(f"  Total ZED Callback Time (Current Frame): {total_processing_time_ms:.2f} ms")
            rospy.loginfo(f"  Estimated Fusion Frame Rate: {1000 / avg_fusion_time:.2f} Hz")
            rospy.loginfo(f"--------------------------------------------------")
        processing_times = [] # Reset for the next interval
        frame_counter = 0
        last_report_time = current_time



def vlp_callback(vlp_pc):
    rospy.loginfo("vlp callback")
    global vlp_depth, vlp_mean, zed_depth_frame_id

    # VLP Preproc
    vlp_pts = msg2pts(vlp_pc)
    vlp_sph_pts_raw = cart_to_sph_pts(vlp_pts[vlp_pts[:, 0] > 0])
    mask = (vlp_sph_pts_raw[:, 2] < ZED_H_ANGLE / 2) & (vlp_sph_pts_raw[:, 2] > -ZED_H_ANGLE / 2)
    vlp_sph_pts = vlp_sph_pts_raw[mask]

    rospy.logdebug("vlp_filtered_pc_p.publish1")

    # --- NEW: PUBLISH FILTERED VLP POINT CLOUD ---
    if len(vlp_sph_pts) > 0: # Ensure there are points to publish
        # Convert filtered spherical points back to Cartesian for publishing
        vlp_filtered_cart_pts = sph_to_cart_pts(vlp_sph_pts.copy())
        vlp_filtered_cart_pts_np = vlp_filtered_cart_pts.get() # CuPy to NumPy

        # Create PointCloud2 message for filtered VLP
        header = vlp_pc.header # Use original VLP header for frame_id and timestamp
        header.frame_id = zed_depth_frame_id
        # It's crucial that the frame_id of this point cloud matches the frame_id of your ZED camera.
        # If your LiDAR frame is different from your camera frame, you'll need a static transform
        # between them in your TF tree for proper visualization in RViz.
        # For now, we'll assume the transform is handled elsewhere or the frame_id is correct.
        # header.frame_id = "velodyne" # Or your LiDAR's base frame if it's not aligned with ZED

        fields = [
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        ]
        vlp_filtered_pc_msg = pc2.create_cloud(header, fields, vlp_filtered_cart_pts_np)
        vlp_filtered_pc_p.publish(vlp_filtered_pc_msg)
        rospy.logdebug("vlp_filtered_pc_p.publish")
    # -----------------------------------------------

    r, theta, phi = vlp_sph_pts.T
    theta = remap(theta, -LiDAR_ANGLE / 2, LiDAR_ANGLE / 2, 3 * ZED_V // 4, ZED_V // 4).astype(cp.int32)
    phi = remap(phi, ZED_V_ANGLE / 2, -ZED_V_ANGLE / 2, 0, ZED_H).astype(cp.int32)

    vlp_mean = cp.mean(vlp_sph_pts[:, 0])

    vlp_depth = cp.zeros((ZED_V, ZED_H), dtype=cp.float32)

    cpx.scatter_add(vlp_depth, (theta, phi), r)


def rgb_callback(msg):
    rospy.loginfo("rgb callback")
    global pg_rgb_msg
    pg_rgb_msg = msg


def camera_info_callback(msg):
    rospy.loginfo("camera_info callback")
    global pg_camera_info_msg
    pg_camera_info_msg = msg


def odom_callback(msg):
    rospy.loginfo("odom callback")
    global pg_odom_msg
    pg_odom_msg = msg


# %%
rospy.Subscriber(VLP_TOPIC, PointCloud2, vlp_callback)
rospy.Subscriber(ZED_RGB_TOPIC, Image, rgb_callback)
rospy.Subscriber(ZED_CAMERA_INFO_TOPIC, CameraInfo, camera_info_callback)
rospy.Subscriber(LOAM_ODOM_TOPIC, Odometry, odom_callback)
rospy.Subscriber(ZED_DEPTH_TOPIC, Image, zed_callback)
rospy.spin()

# %% Performance
import csv
import logging
import os
import time

# %%
import cupy as cp
import cupyx.scipy.ndimage
import cv2
import message_filters
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

VLP_TOPIC = "/cumulative_origin_point_cloud"

LOAM_ODOM_TOPIC = '/islam/vlp_odom'

VLP_DEPTH_TOPIC = "/islam/vlp_depth"
PG_DEPTH_TOPIC = "/islam/pg_depth"
PG_CAMERA_INFO_TOPIC = '/islam/pg_camera_info'
PG_RGB_TOPIC = '/islam/pg_rgb'
PG_ODOM_TOPIC = '/islam/pg_odom'
PG_FUSED_PC_TOPIC = "/islam/pg_fused_pointcloud"
ZED_PC_TOPIC = "/islam/zed_pointcloud"
VLP_FILTERED_PC_TOPIC = "/islam/vlp_filtered_pointcloud"
VLP_DEBUG_PC_TOPIC = "/islam/vlp_debug_pointcloud"

# %% Algorithm parameters
CURRENT_NCUTOFF = 0.4
CURRENT_THRESHOLD = 10
MORTAL_ROWS_TOP = 320
MORTAL_ROWS_BOTTOM = 320

# %%
# ZED_V = 376
# ZED_H = 672
ZED_H_ANGLE = 102
ZED_V_ANGLE = 56

LiDAR_V = 16
LiDAR_ANGLE = 30.2  # 0.2 degree precaution TODO: Check if precaution is required and proper


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
ROS_DATATYPE_TO_NP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64
}


def msg2pts(msg):
    # Map the msg.fields to get offsets and dtypes for x, y, z
    wanted_fields = ["x", "y", "z"]
    field_map = {f.name: f for f in msg.fields if f.name in wanted_fields}
    dtype_list = []

    for name in wanted_fields:
        field = field_map[name]
        np_dtype = ROS_DATATYPE_TO_NP[field.datatype]
        dtype_list.append((name, np_dtype))

    # Create structured dtype with correct offsets
    offsets = [field_map[name].offset for name in wanted_fields]
    max_offset = max(offsets)
    point_step = msg.point_step

    # Create dummy structured dtype with padding
    structured_dtype = np.dtype({
        'names': wanted_fields,
        'formats': [ROS_DATATYPE_TO_NP[field_map[n].datatype] for n in wanted_fields],
        'offsets': offsets,
        'itemsize': point_step
    })

    # Interpret raw data
    np_pts = np.frombuffer(msg.data, dtype=structured_dtype, count=msg.width * msg.height)

    # Stack as float32 and convert to CuPy
    xyz = np.stack([np_pts[name].astype(np.float32) for name in wanted_fields], axis=-1)
    return cp.asarray(xyz)


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
    mask = ~cp.isnan(vlp_depth)
    filtered = zed_depth
    filtered[mask] = vlp_depth[mask]

    while threshold > 0:
        filtered = lpf(filtered, ncutoff)
        filtered[mask] = vlp_depth[mask]

        threshold -= 1

    return filtered


# %%
def remap(old_value, old_min, old_max, new_min, new_max):
    """
    Function to map a value from one range to another
    :param old_value:
    :param old_min:
    :param old_max:
    :param new_min:
    :param new_max:
    :return:
    """
    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min) * new_range) / old_range) + new_min
    return new_value


# %% Faster PC creation from NP
def create_cloud_from_np(header, fields, np_array):
    """
    Fast version of create_cloud, using NumPy vectorized byte representation.
    Assumes np_array is (N, 3) float32 for (x, y, z).
    """

    # Flatten the array to 1D byte representation
    data = np_array.astype(np.float32).tobytes()

    cloud_msg = PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = 1
    cloud_msg.width = np_array.shape[0]
    cloud_msg.fields = fields
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 12  # 3 floats * 4 bytes
    cloud_msg.row_step = cloud_msg.point_step * np_array.shape[0]
    cloud_msg.is_dense = True
    cloud_msg.data = data

    return cloud_msg


# %%
def inpaint_depth_opencv(cu_img):
    img = cu_img.get()
    mask = ~np.isfinite(img)

    # OpenCV needs 8-bit single-channel mask
    mask = mask.astype(np.uint8) * 255

    # Normalize for OpenCV (must be 8-bit or float32)
    img32 = img.astype(np.float32)

    inpainted = cv2.inpaint(img32, mask, inpaintRadius=3, flags=cv2.INPAINT_NS)

    return cp.asarray(inpainted)

# %%
rospy.init_node('sf', anonymous=True)
rospy.set_param('/rosgraph/log_level', logging.DEBUG)
rospy.set_param('/use_sim_time', True)

# Global variables for performance logging
processing_times = []
frame_counter = 0
last_report_time = time.time()
report_interval_frames = 100  # Report every 100 frames
report_interval_seconds = 5  # Report every 5 seconds

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
    'VLP_PreprocTime_ms',
    'msg_to_pts_time_ms',
    'cart_to_pts_time_ms',
    'remap_time_ms',
    'vlp_mean_time_ms',
    'scatter_time_ms',
    'VLP_FilteredTime_ms',
    'sph_to_cart_pts_time_ms',
    'filtered_publish_time_ms',
    'ZED_PC_Time_ms',
    'depth_to_cart_time_ms',
    'pts_to_pc_time_ms',
    'cp_to_np_time_ms',
    'pc_to_msg_time_ms',
    'TotalCallbackTime_ms',
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
vlp_depth_p = rospy.Publisher(VLP_DEPTH_TOPIC, Image, queue_size=50)
pg_depth_p = rospy.Publisher(PG_DEPTH_TOPIC, Image, queue_size=50)
pg_camera_info_p = rospy.Publisher(PG_CAMERA_INFO_TOPIC, CameraInfo, queue_size=50)
pg_rgb_p = rospy.Publisher(PG_RGB_TOPIC, Image, queue_size=50)
pg_odom_p = rospy.Publisher(PG_ODOM_TOPIC, Odometry, queue_size=10)
pg_fused_pc_p = rospy.Publisher(PG_FUSED_PC_TOPIC, PointCloud2, queue_size=10)
zed_pc_p = rospy.Publisher(ZED_PC_TOPIC, PointCloud2, queue_size=10)
vlp_filtered_pc_p = rospy.Publisher(VLP_FILTERED_PC_TOPIC, PointCloud2, queue_size=10)
vlp_debug_pc_p = rospy.Publisher(VLP_DEBUG_PC_TOPIC, PointCloud2, queue_size=10)

bridge = CvBridge()

zed_img_init = rospy.wait_for_message(ZED_DEPTH_TOPIC, Image)
ZED_V, ZED_H = cp.array(bridge.imgmsg_to_cv2(zed_img_init, "32FC1")).shape
# --- Store ZED's initial frame_id ---
zed_depth_frame_id = 'map'  # zed_img_init.header.frame_id
rospy.loginfo(f"Detected ZED Depth Frame ID: {zed_depth_frame_id}")

cp_to_np_time_ms = 0
pc_to_msg_time_ms = 0


# %% Subscriber and Synchronizer Setup

def synchronized_callback(
        zed_img_msg: Image,
        vlp_pc_msg: PointCloud2,
        pg_rgb_msg: Image,
        pg_camera_info_msg: CameraInfo,
        pg_odom_msg: Odometry
):
    """
    Callback function for the synchronized callback
    :param zed_img_msg:
    :param vlp_pc_msg:
    :param pg_rgb_msg:
    :param pg_camera_info_msg:
    :param pg_odom_msg:
    :return:
    """
    global processing_times, frame_counter, last_report_time, cp_to_np_time_ms, pc_to_msg_time_ms

    rospy.loginfo("synchronized_callback")

    # Record start time for the entire processing
    total_start_time = time.time()

    # %% VLP Preproc
    def cart_pts_to_depth_image(cart_pts, camera_info_msg, image_shape):
        """
        Vectorized projection of 3D points to a depth image using camera intrinsics.
        Args:
            cart_pts: CuPy array (N, 3), in camera frame with X=forward
            camera_info_msg: sensor_msgs/CameraInfo
            image_shape: (H, W)
        Returns:
            depth_image: CuPy array (H, W) with float32 values
        """

        fx = camera_info_msg.K[0]
        fy = camera_info_msg.K[4]
        cx = camera_info_msg.K[2]
        cy = camera_info_msg.K[5]

        # Convert from your axis convention (New X = Old Z_camera, etc.)
        Z = cart_pts[:, 0]
        X = -cart_pts[:, 1]
        Y = -cart_pts[:, 2]

        # Image coordinates
        u = cp.round((X * fx) / Z + cx).astype(cp.int32)
        v = cp.round((Y * fy) / Z + cy).astype(cp.int32)

        H, W = image_shape
        valid = (u >= 0) & (u < W) & (v >= 0) & (v < H) & (Z > 0) & cp.isfinite(Z)
        u = u[valid]
        v = v[valid]
        Z = Z[valid]

        # Compute flat indices for fast scatter
        flat_idx = v * W + u
        sorted_idx = cp.argsort(flat_idx)

        flat_idx = flat_idx[sorted_idx]
        Z_sorted = Z[sorted_idx]

        # Find first occurrence of each (u, v)
        unique_idx, first_pos = cp.unique(flat_idx, return_index=True)
        Z_min = Z_sorted[first_pos]

        # Reconstruct 2D depth image
        depth_img_flat = cp.full(H * W, cp.nan, dtype=cp.float32)
        depth_img_flat[unique_idx] = Z_min
        depth_img = depth_img_flat.reshape(H, W)

        return depth_img

    vlp_preproc_start_time = time.time()

    msg_to_pts_start_time = time.time()
    vlp_pts = msg2pts(vlp_pc_msg)
    msg_to_pts_end_time = time.time()
    msg_to_pts_time_ms = (msg_to_pts_end_time - msg_to_pts_start_time) * 1000

    cart_to_pts_start_time = time.time()
    vlp_sph_pts_raw = cart_to_sph_pts(vlp_pts[vlp_pts[:, 0] > 0])
    cart_to_pts_end_time = time.time()
    cart_to_pts_time_ms = (cart_to_pts_end_time - cart_to_pts_start_time) * 1000

    remap_start_time = time.time()
    mask = (vlp_sph_pts_raw[:, 2] < ZED_H_ANGLE / 2) & (vlp_sph_pts_raw[:, 2] > -ZED_H_ANGLE / 2)
    vlp_sph_pts = vlp_sph_pts_raw[mask]
    # r, theta, phi = vlp_sph_pts.T
    # theta = remap(theta, -LiDAR_ANGLE / 2, LiDAR_ANGLE / 2, 3 * ZED_V // 4, ZED_V // 4).astype(cp.int32)
    # phi = remap(phi, ZED_V_ANGLE / 2, -ZED_V_ANGLE / 2, 0, ZED_H).astype(cp.int32)
    remap_end_time = time.time()
    remap_time_ms = (remap_end_time - remap_start_time) * 1000

    vlp_mean_start_time = time.time()
    vlp_mean = cp.mean(vlp_sph_pts[:, 0])
    vlp_mean_end_time = time.time()
    vlp_mean_time_ms = (vlp_mean_end_time - vlp_mean_start_time) * 1000

    scatter_start_time = time.time()
    # vlp_depth = cp.zeros((ZED_V, ZED_H), dtype=cp.float32)
    # cpx.scatter_add(vlp_depth, (theta, phi), r)
    vlp_depth = cart_pts_to_depth_image(vlp_pts, pg_camera_info_msg, (ZED_V, ZED_H))
    scatter_end_time = time.time()
    scatter_time_ms = (scatter_end_time - scatter_start_time) * 1000

    vlp_preproc_end_time = time.time()
    vlp_preproc_time_ms = (vlp_preproc_end_time - vlp_preproc_start_time) * 1000

    # %% Publish VLP depth image
    # Publish Image
    vlp_depth_msg = bridge.cv2_to_imgmsg(vlp_depth.get())
    vlp_depth_msg.header.stamp = zed_img_msg.header.stamp
    vlp_depth_p.publish(vlp_depth_msg)
    rospy.logwarn("vlp_depth published")

    # %% Publish filtered vlp point cloud
    vlp_filtered_start_time = time.time()
    sph_to_cart_pts_time_ms = 0
    filtered_publish_time_ms = 0

    if len(vlp_sph_pts) > 0:  # Ensure there are points to publish
        # %% Convert filtered spherical points back to Cartesian for publishing
        sph_to_cart_pts_start_time = time.time()
        vlp_filtered_cart_pts = sph_to_cart_pts(vlp_sph_pts.copy())
        vlp_filtered_cart_pts_np = vlp_filtered_cart_pts.get()  # CuPy to NumPy
        sph_to_cart_pts_end_time = time.time()
        sph_to_cart_pts_time_ms = (sph_to_cart_pts_end_time - sph_to_cart_pts_start_time) * 1000

        # %% Create PointCloud2 message for filtered VLP
        filtered_publish_start_time = time.time()
        header = vlp_pc_msg.header  # Use original VLP header for frame_id and timestamp
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
        # vlp_filtered_pc_msg = pc2.create_cloud(header, fields, vlp_filtered_cart_pts_np)
        vlp_filtered_pc_msg = create_cloud_from_np(header, fields, vlp_filtered_cart_pts_np)
        vlp_filtered_pc_p.publish(vlp_filtered_pc_msg)
        filtered_publish_end_time = time.time()
        filtered_publish_time_ms = (filtered_publish_end_time - filtered_publish_start_time) * 1000

        rospy.logdebug("vlp_filtered_pc_p.publish")

    vlp_filtered_end_time = time.time()
    vlp_filtered_time_ms = (vlp_filtered_end_time - vlp_filtered_start_time) * 1000

    # %% ZED Preproc
    zed_depth = cp.array(bridge.imgmsg_to_cv2(zed_img_msg, "32FC1"))

    # Create mask for NaNs and out-of-range values
    mask_nan_zed = cp.isnan(zed_depth)
    mask_inf_zed = cp.isinf(zed_depth)

    # Combine masks
    filled_mask = mask_nan_zed | mask_inf_zed

    # Create filled version of the depth image
    zed_depth = zed_depth.copy()
    zed_depth = inpaint_depth_opencv(zed_depth)
    # TODO: Max value of ZED?
    zed_depth[~cp.isfinite(zed_depth)] = 40
    # zed_depth[filled_mask] = 40

    # %% Sensor Fusion
    fusion_start_time = time.time()  # Start timing only the fusion part

    zed_depth_cropped = zed_depth[MORTAL_ROWS_TOP:-MORTAL_ROWS_BOTTOM, :].copy()
    vlp_depth_cropped = vlp_depth[MORTAL_ROWS_TOP:-MORTAL_ROWS_BOTTOM, :].copy()

    pg_depth_cropped = pg(zed_depth_cropped.copy(), vlp_depth_cropped.copy(), ncutoff=CURRENT_NCUTOFF, threshold=CURRENT_THRESHOLD)

    zed_depth = cp.pad(
        zed_depth_cropped,
        ((MORTAL_ROWS_TOP, MORTAL_ROWS_BOTTOM), (0, 0)),  # pad only at the bottom
        mode='constant',
        constant_values=cp.nan
    )
    vlp_depth = cp.pad(
        vlp_depth_cropped,
        ((MORTAL_ROWS_TOP, MORTAL_ROWS_BOTTOM), (0, 0)),  # pad only at the bottom
        mode='constant',
        constant_values=cp.nan
    )

    pg_depth = cp.pad(
        pg_depth_cropped,
        ((MORTAL_ROWS_TOP, MORTAL_ROWS_BOTTOM), (0, 0)),  # pad only at the bottom
        mode='constant',
        constant_values=cp.nan
    )

    fusion_end_time = time.time()  # End timing fusion part
    fusion_time_ms = (fusion_end_time - fusion_start_time) * 1000

    processing_times.append(fusion_time_ms)
    frame_counter += 1

    # Remove filled pixels
    pg_depth[filled_mask] = cp.nan

    # Publish Image
    pg_depth_msg = bridge.cv2_to_imgmsg(pg_depth.get())
    pg_depth_msg.header.stamp = zed_img_msg.header.stamp
    pg_depth_p.publish(pg_depth_msg)
    rospy.logwarn("pg_depth published")

    # %% Convert Fused Depth to PointCloud2 and Publish
    # Need camera intrinsics for this. Assume pg_camera_info_msg is already populated.
    # Convert depth image to Cartesian points first
    # This part can be computationally heavy, consider if you need it for every frame.
    # If not, you might publish it less frequently or only for debugging.

    zed_pc_start_time = time.time()
    depth_to_cart_time_ms = 0
    pts_to_pc_time_ms = 0

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
            # cart_pts = cp.stack((X, Y, Z), axis=-1).reshape(-1, 3)

            # Filter out invalid points (e.g., where Z is 0 or NaN)
            valid_mask = (Z.flatten() > 0) & cp.isfinite(Z.flatten())

            # Apply the rotation:
            # New X (forward) = Old Z_camera
            # New Y (left)    = Old (-X_camera)
            # New Z (up)      = Old (-Y_camera)
            cart_pts = cp.stack((Z, -X, -Y), axis=-1).reshape(-1, 3)

            cart_pts = cart_pts[valid_mask]
            return cart_pts

        depth_to_cart_start_time = time.time()
        fused_cart_pts = depth_to_cart_pts(pg_depth, camera_info_msg=pg_camera_info_msg)
        zed_cart_pts = depth_to_cart_pts(zed_depth, camera_info_msg=pg_camera_info_msg)
        vlp_cart_pts = depth_to_cart_pts(vlp_depth, camera_info_msg=pg_camera_info_msg)
        depth_to_cart_end_time = time.time()
        depth_to_cart_time_ms = (depth_to_cart_end_time - depth_to_cart_start_time) * 1000

        def cart_pts_to_pc_msg(cart_pts, header):
            # Convert CuPy array to NumPy for pc2.create_cloud
            global cp_to_np_time_ms, pc_to_msg_time_ms
            cp_to_np_start_time = time.time()
            cart_pts_np = cart_pts.get()
            cp_to_np_end_time = time.time()
            cp_to_np_time_ms = (cp_to_np_end_time - cp_to_np_start_time) * 1000

            fields = [
                pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            ]

            pc_to_msg_start_time = time.time()
            # pc_msg = pc2.create_cloud(header, fields, cart_pts_np)
            pc_msg = create_cloud_from_np(header, fields, cart_pts_np)
            pc_to_msg_end_time = time.time()
            pc_to_msg_time_ms = (pc_to_msg_end_time - pc_to_msg_start_time) * 1000
            return pc_msg

        # Create PointCloud2 message
        header = zed_img_msg.header
        header.frame_id = zed_depth_frame_id  # pg_camera_info_msg.header.frame_id # Ensure frame_id is correct (usually camera_link/optical_frame)

        pts_to_pc_start_time = time.time()
        pg_fused_pc_msg = cart_pts_to_pc_msg(fused_cart_pts, header)
        zed_pc_msg = cart_pts_to_pc_msg(zed_cart_pts, header)
        vlp_pc_msg = cart_pts_to_pc_msg(vlp_cart_pts, header)
        pts_to_pc_end_time = time.time()
        pts_to_pc_time_ms = (pts_to_pc_end_time - pts_to_pc_start_time) * 1000

        pg_fused_pc_p.publish(pg_fused_pc_msg)
        zed_pc_p.publish(zed_pc_msg)
        vlp_debug_pc_p.publish(vlp_pc_msg)
        rospy.loginfo("publish_pg")
    else:
        rospy.logwarn("CameraInfo not received yet or invalid, skipping fused point cloud publication.")
        rospy.logwarn_throttle(5, "CameraInfo not received yet or invalid, skipping fused point cloud publication.")

    zed_pc_end_time = time.time()
    zed_pc_time_ms = (zed_pc_end_time - zed_pc_start_time) * 1000

    # %% Publish aux info
    pg_rgb_msg.header.stamp = zed_img_msg.header.stamp
    pg_camera_info_msg.header.stamp = zed_img_msg.header.stamp
    pg_odom_msg.header.stamp = zed_img_msg.header.stamp
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
        vlp_preproc_time_ms,
        msg_to_pts_time_ms,
        cart_to_pts_time_ms,
        remap_time_ms,
        vlp_mean_time_ms,
        scatter_time_ms,
        vlp_filtered_time_ms,
        sph_to_cart_pts_time_ms,
        filtered_publish_time_ms,
        zed_pc_time_ms,
        depth_to_cart_time_ms,
        pts_to_pc_time_ms,
        cp_to_np_time_ms,
        pc_to_msg_time_ms,
        total_processing_time_ms,
        CURRENT_NCUTOFF,  # Log ncutoff
        CURRENT_THRESHOLD  # Log threshold
    ])
    log_file.flush()  # Ensure data is written to disk immediately

    # Report performance metrics periodically
    current_time = time.time()
    if frame_counter >= report_interval_frames or (current_time - last_report_time) >= report_interval_seconds:
        if processing_times:  # Ensure there's data to calculate
            avg_fusion_time = sum(processing_times) / len(processing_times)
            max_fusion_time = max(processing_times)
            min_fusion_time = min(processing_times)
            rospy.loginfo(f"--- Performance Report (Last {len(processing_times)} frames) ---")
            rospy.loginfo(
                f"  Parameters: ncutoff={CURRENT_NCUTOFF}, threshold={CURRENT_THRESHOLD}")  # Log params in console report
            rospy.loginfo(f"  Avg Fusion Time: {avg_fusion_time:.2f} ms")
            rospy.loginfo(f"  Max Fusion Time: {max_fusion_time:.2f} ms")
            rospy.loginfo(f"  Min Fusion Time: {min_fusion_time:.2f} ms")
            rospy.loginfo(f"  Total ZED Callback Time (Current Frame): {total_processing_time_ms:.2f} ms")
            rospy.loginfo(f"  Estimated Fusion Frame Rate: {1000 / avg_fusion_time:.2f} Hz")
            rospy.loginfo(f"--------------------------------------------------")
        processing_times = []  # Reset for the next interval
        frame_counter = 0
        last_report_time = current_time

    # --- DEBUG PLOTTING SECTION ---
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    axs[0].imshow(zed_depth.get(), cmap='plasma')
    axs[0].set_title("ZED Depth (Original)")
    axs[1].imshow(vlp_depth.get(), cmap='inferno')
    axs[1].set_title("VLP Depth")
    axs[2].imshow(pg_depth.get(), cmap='viridis')
    axs[2].set_title("Fused PG Depth")

    for ax in axs:
        ax.axis('off')

    plt.tight_layout()
    plt.show()  # <-- This blocks until you close the plot window


# Create message_filters subscribers
depth_sub = message_filters.Subscriber(ZED_DEPTH_TOPIC, Image)
vlp_sub = message_filters.Subscriber(VLP_TOPIC, PointCloud2)
rgb_sub = message_filters.Subscriber(ZED_RGB_TOPIC, Image)
cam_info_sub = message_filters.Subscriber(ZED_CAMERA_INFO_TOPIC, CameraInfo)
odom_sub = message_filters.Subscriber(LOAM_ODOM_TOPIC, Odometry)

# Create an ApproximateTimeSynchronizer
# queue_size: How many sets of messages to buffer
# slop: Maximum allowed time difference between messages in a set (seconds)
# You may need to tune the 'slop' value based on your sensor synchronization and message arrival rates.
# A larger slop allows more desynchronized messages but increases latency and potential for bad associations.
# A smaller slop might cause you to miss many synchronized sets if messages are not perfectly aligned.
ats = message_filters.ApproximateTimeSynchronizer(
    [depth_sub, vlp_sub, rgb_sub, cam_info_sub, odom_sub],
    # [rgb_sub, cam_info_sub],
    queue_size=10,  # Adjust as needed
    slop=0.1,
    reset=True
)

# Register the synchronized callback
ats.registerCallback(synchronized_callback)

# %%
rospy.spin()

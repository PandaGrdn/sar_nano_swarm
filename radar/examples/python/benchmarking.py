from filtering import *
from collections import defaultdict
from ekf import VelocityEKF  # or paste the class directly into this file
import numpy as np
from benchmarking_funcs import *

SEQ_DIR='/Users/monika/Downloads/radar/data/coloradar/kitti/2_23_2021_edgar_classroom_run4'
CALIB_DIR='/Users/monika/Downloads/radar/data/coloradar/calib'

# thresholds
radar_num_threshold = 7
ekf = VelocityEKF(measurement_noise=0.3)

radar_map = build_radar_frame_map()
radar_dict = {frame: pts for frame, pts in radar_map}
radar_timestamps, poses, indicies = get_groundtruth_adjusted_radar(SEQ_DIR, CALIB_DIR)
timestamps = []
gt_poses = []
indicies = indicies[0:30]
for idx in indicies:
    timestamps.append(radar_timestamps[idx])
    gt_poses.append(matrix_to_pose_dict(poses[idx]))

print(list(radar_timestamps[0:30]))
radar_dict = {k: radar_dict[k] for k in indicies}

# # run on the first timestamp
prev_cluster_centers = []
prev_cluster_num_points = []
prev_timestamp = timestamps[0]
radar_points_0 = radar_dict[indicies[0]]
prev_cluster_centers, prev_cluster_num_points = clustering(radar_points_0)

prev_pose_gt = gt_poses[0]

print(list(timestamps))
# run on remainder of frames and calcualte velocity then benchmark it:
for timestamp_idx in indicies[1:]:
    radar_points = radar_dict[timestamp_idx]
    timestamp = timestamps[timestamp_idx]
    print("timestamp: ", timestamps[timestamp_idx], " ", prev_timestamp)
    
    vel_estimate, bias, omega, gyro_bias, prev_cluster_centers, prev_cluster_num_points = radar_landmarking(ekf, prev_cluster_centers, prev_timestamp, timestamp, radar_points, radar_num_threshold=7)
    pos_delta, rot_delta = pose_delta(prev_pose_gt, gt_poses[timestamp_idx])
    
    velocity_gt = pos_delta / (timestamp - prev_timestamp)
    omega_gt = rot_delta.as_rotvec() / (timestamp - prev_timestamp)
    
    print(vel_estimate, "  |||  ", velocity_gt)
    prev_timestamp = timestamp
    z = input("___________")
    
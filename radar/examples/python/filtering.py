from dataset_loaders import *
import math
import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation, Slerp
import argparse
from time import time
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN

def interpolate_poses(src_poses, src_stamps, tgt_stamps):

  src_start_idx = 0
  tgt_start_idx = 0
  src_end_idx = len(src_stamps) - 1
  tgt_end_idx = len(tgt_stamps) - 1

  # ensure first source timestamp is immediately before first target timestamp
  while tgt_start_idx < tgt_end_idx and tgt_stamps[tgt_start_idx] < src_stamps[src_start_idx]:
    tgt_start_idx += 1

  # ensure last source timestamp is immediately after last target timestamp
  while tgt_end_idx > tgt_start_idx and tgt_stamps[tgt_end_idx] > src_stamps[src_end_idx]:
    tgt_end_idx -= 1

  # iterate through target timestamps, 
  # interpolating a pose for each as a 4x4 transformation matrix
  tgt_idx = tgt_start_idx
  src_idx = src_start_idx
  tgt_poses = []
  while tgt_idx <= tgt_end_idx and src_idx <= src_end_idx:
    # find source timestamps bracketing target timestamp
    while src_idx + 1 <= src_end_idx and src_stamps[src_idx + 1] < tgt_stamps[tgt_idx]:
      src_idx += 1

    # get interpolation coefficient
    c = ((tgt_stamps[tgt_idx] - src_stamps[src_idx]) 
          / (src_stamps[src_idx+1] - src_stamps[src_idx]))

    # interpolate position
    pose = np.eye(4)
    pose[:3,3] = ((1.0 - c) * src_poses[src_idx]['position'] 
                        + c * src_poses[src_idx+1]['position'])

    # interpolate orientation
    r_src = Rotation.from_quat([src_poses[src_idx]['orientation'],
                            src_poses[src_idx+1]['orientation']])
    slerp = Slerp([0,1],r_src)
    pose[:3,:3] = slerp([c])[0].as_matrix()

    tgt_poses.append(pose)

    # advance target index
    tgt_idx += 1

  tgt_indices = range(tgt_start_idx, tgt_end_idx + 1)
  return tgt_poses, tgt_indices


# downsamples a pointcloud for faster plotting using a voxel grid
# output pointcloud will have at most one point in a given voxel
# param[in] in_pcl: the pointcloud to be downsampled
# param[in] vox_size: the voxel size
# return out_pcl: the downsampled pointcloud
def downsample_pointcloud(in_pcl, vox_size):
  _, idx = np.unique((in_pcl[:,:3] / vox_size).round(),return_index=True,axis=0)
  out_pcl = in_pcl[idx,:]
  return out_pcl

# converts a bin location in a polar-formatted heatmap to a point in 
# cartesian space defined in meters
# param[in] r_bin: range bin index
# param[in] az_bin: azimuth bin index
# param[in] el_bin: elevation bin index
# param[in] params: heatmap parameters for the sensor
# return point: the point in cartesian coordinates
def polar_to_cartesian(r_bin, az_bin, el_bin, params):
  point = np.zeros(3)
  point[0] = (r_bin * params['range_bin_width'] 
              * math.cos(params['elevation_bins'][el_bin]) 
              * math.cos(params['azimuth_bins'][az_bin]))
  point[1] = (r_bin * params['range_bin_width']
              * math.cos(params['elevation_bins'][el_bin])
              * math.sin(params['azimuth_bins'][az_bin]))
  point[2] = (r_bin * params['range_bin_width']
              * math.sin(params['elevation_bins'][el_bin]))
  return point

# calculates point locations in the sensor frame for plotting heatmaps
# param[in] params: heatmap parameters for the sensor
# return pcl: the heatmap point locations
def get_heatmap_points(params, min_range=1):

  # transform range-azimuth-elevation heatmap to pointcloud
  pcl = np.zeros([params['num_elevation_bins'],
                  params['num_azimuth_bins'],
                  params['num_range_bins'] - min_range,
                  5])

  for range_idx in range(params['num_range_bins'] - min_range):
    for az_idx in range(params['num_azimuth_bins']):
      for el_idx in range(params['num_elevation_bins']):
        pcl[el_idx,az_idx,range_idx,:3] = polar_to_cartesian(range_idx + min_range, az_idx, el_idx, params)

  pcl = pcl.reshape(-1,5)
  return pcl

# performs a rigid transformation on a pointcloud
# param[in] pcl: the input pointcloud to be transformed
# param[in] T: the 4x4 rigid transformation matrix
# return out_points: the transformed pointcloud
def transform_pcl(pcl, T):
  in_points = pcl[:,:3]
  in_points = np.concatenate((in_points,np.ones((in_points.shape[0],1))), axis=1)
  out_points = np.dot(T,np.transpose(in_points))
  out_points = np.transpose(out_points[:3,:])
  if pcl.shape[1] > 3:
    out_points = np.concatenate((out_points,pcl[:,3:]), axis=1)
  return out_points

# seq_dir: directory of the sequence
# calib_dir: directory of the calibration data
# threshold: intensity threshold for plotting heatmap points
# min_range: if plotting heatmaps, minimum range bin to start plotting

def filter_radar_by_lidar(radar_points, lidar_points, max_dist=1):
    """
    Keep only radar points that have a nearby lidar point,
    using lidar as a validity mask for real surfaces.

    radar_points: (N, 3) array of radar xyz points
    lidar_points: (M, 3) array of lidar xyz points, same coordinate frame
    max_dist: max distance (meters) for a radar point to be considered "confirmed"

    Returns: boolean mask (True = keep)
    """
    lidar_tree = cKDTree(lidar_points[:, :3])
    distances, _ = lidar_tree.query(radar_points[:, :3], k=1)
    return distances <= max_dist
 
def build_radar_frame_map(seq_dir='/Users/monika/Downloads/radar/data/coloradar/kitti/2_23_2021_edgar_classroom_run4',
                            calib_dir='/Users/monika/Downloads/radar/data/coloradar/calib',
                            threshold=0.004, min_range=0):
   """
   Builds a structure mapping each radar frame number to its processed point cloud.

   Returns: list of (frame_number, radar_points) tuples,
            where radar_points is an (N, 6) array [x, y, z, intensity, doppler, timestamp]
   """
   all_radar_params = get_cascade_params(calib_dir)
   radar_params = all_radar_params['heatmap']
   gt_params = get_groundtruth_params()

   radar_params['T_bs'] = np.eye(4)
   radar_params['T_bs'][:3,3] = radar_params['translation']
   radar_params['T_bs'][:3,:3] = Rotation.from_quat(radar_params['rotation']).as_matrix()

   radar_timestamps = get_timestamps(seq_dir, radar_params)
   gt_timestamps = get_timestamps(seq_dir, gt_params)

   gt_poses = get_groundtruth(seq_dir)

   radar_gt, radar_indices = interpolate_poses(gt_poses, gt_timestamps, radar_timestamps)

   radar_pc_precalc = get_heatmap_points(radar_params, min_range)

   radar_frame_map = []

   for i, radar_idx in enumerate(radar_indices[0:30]):
     R_wb = np.array(radar_gt[i])
     t_wb = np.array(R_wb[:3,3])
     R_wb[:3,3] = 0.0

     radar_hm = get_heatmap(radar_idx, seq_dir, radar_params)
     radar_pc_precalc[:,3:] = radar_hm[:,:,min_range:,:].reshape(-1,2)
     radar_pc_local = downsample_pointcloud(radar_pc_precalc, 0.3)

     radar_pc_local[:,3] -= radar_pc_local[:,3].min()
     radar_pc_local[:,3] /= radar_pc_local[:,3].max()
     radar_pc_local = radar_pc_local[radar_pc_local[:,3] > threshold]

     if radar_pc_local.shape[0] == 0:
       continue  # nothing survived thresholding this frame

     radar_pc_local[:,3] -= radar_pc_local[:,3].min()
     radar_pc_local[:,3] /= radar_pc_local[:,3].max()

     T_ws = np.dot(R_wb, radar_params['T_bs'])
     radar_pc = transform_pcl(radar_pc_local, T_ws)

     # NEW: append the frame's timestamp as an extra column for every point
     frame_timestamp = radar_timestamps[radar_idx]
     timestamp_col = np.full((radar_pc.shape[0], 1), frame_timestamp)
     radar_pc = np.hstack([radar_pc, timestamp_col])

     radar_frame_map.append((radar_idx, radar_pc))

   return radar_frame_map
 
def clustering(radar_points, eps=0.5, min_samples=5):
    """
    Clusters radar points using DBSCAN.

    radar_points: (N, 3) array of radar xyz points
    eps: max distance between points to be considered in the same cluster
    min_samples: minimum number of points to form a cluster

    Returns: labels for each point (-1 = noise)
    """
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(radar_points[:, :3])
    return clustering.labels_
  
import matplotlib.pyplot as plt
import matplotlib.cm as cm

def plot_clusters(cluster_dict, cluster_centers, title="Radar Clusters"):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection='3d')

    # exclude noise (-1) from the color-mapped cluster list, plot it separately in gray
    cluster_labels = sorted([l for l in cluster_dict.keys() if l != -1])
    colors = matplotlib.colormaps['tab20'].resampled(max(len(cluster_labels), 1))

    for i, label in enumerate(cluster_labels):
        points = np.array(cluster_dict[label])
        color = colors(i)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                   color=color, s=15, alpha=0.6, label=f'cluster {label}')

        # highlight the centroid: bigger marker, black edge, star shape
        if label in cluster_centers:
            center = cluster_centers[label]
            ax.scatter(center[0], center[1], center[2],
                       color=color, s=250, marker='*',
                       edgecolors='black', linewidths=1.5, zorder=10)

    # plot noise points in gray, if present
    if -1 in cluster_dict:
        noise_points = np.array(cluster_dict[-1])
        ax.scatter(noise_points[:, 0], noise_points[:, 1], noise_points[:, 2],
                   color='lightgray', s=5, alpha=0.3, label='noise')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=8)
    plt.tight_layout()
    plt.show()

import numpy as np

def log_likelihood_distance(point1, point2, covariance=None):
    """
    Computes the log-likelihood distance between two 3D points, treating
    point1 as the mean of a Gaussian distribution and evaluating the
    negative log-likelihood of point2 under that distribution.

    This is the Mahalanobis distance plus normalization terms (i.e. the
    full negative log-likelihood of a multivariate Gaussian), which is
    more principled than raw Euclidean distance when measurement
    uncertainty differs across dimensions (e.g. radar range vs. angle
    accuracy).

    Parameters
    ----------
    point1 : array-like, shape (3,)
        The reference point (treated as the distribution's mean).
    point2 : array-like, shape (3,)
        The point being evaluated against that distribution.
    covariance : array-like, shape (3, 3), optional
        Covariance matrix representing uncertainty. Defaults to identity
        (i.e. reduces to squared Euclidean distance / 2, up to constants).

    Returns
    -------
    float
        The negative log-likelihood (lower = more similar / more likely
        the two points represent the same underlying object).
    """
    p1 = np.asarray(point1, dtype=float)
    p2 = np.asarray(point2, dtype=float)
    diff = p2 - p1

    if covariance is None:
        covariance = np.eye(3)
    else:
        covariance = np.asarray(covariance, dtype=float)

    cov_inv = np.linalg.inv(covariance)
    mahalanobis_sq = diff.T @ cov_inv @ diff

    d = len(p1)
    log_det_cov = np.log(np.linalg.det(covariance))
    log_likelihood = -0.5 * (mahalanobis_sq + log_det_cov + d * np.log(2 * np.pi))

    # negative log-likelihood as a "distance" (lower = closer/more likely)
    return -log_likelihood


import numpy as np

def estimate_velocity(point1, point2, timestamp1, timestamp2):
    """
    Estimates velocity between two 3D points given their timestamps.

    Parameters
    ----------
    point1 : array-like, shape (3,)
        Position at timestamp1 (e.g. a cluster centroid at frame i).
    point2 : array-like, shape (3,)
        Position at timestamp2 (e.g. the same landmark's centroid at frame i+1).
    timestamp1 : float
        Time (seconds) corresponding to point1.
    timestamp2 : float
        Time (seconds) corresponding to point2.

    Returns
    -------
    velocity_vector : np.ndarray, shape (3,)
        Velocity components [vx, vy, vz] in units/second.
    speed : float
        Scalar speed (magnitude of the velocity vector).

    Raises
    ------
    ValueError
        If timestamps are equal or out of order (zero/negative time delta).
    """
    p1 = np.asarray(point1, dtype=float)
    p2 = np.asarray(point2, dtype=float)

    dt = timestamp2 - timestamp1
    if dt <= 0:
        raise ValueError(f"Invalid time delta ({dt}); timestamp2 must be greater than timestamp1.")

    displacement = p2 - p1
    velocity_vector = displacement / dt
    speed = np.linalg.norm(velocity_vector)

    return velocity_vector, speed

def get_IMU_data(seq_dir='/Users/monika/Downloads/radar/data/coloradar/kitti/2_23_2021_edgar_classroom_run4',
                 calib_dir='/Users/monika/Downloads/radar/data/coloradar/calib'):
    """
    Retrieves IMU data from the specified sequence directory.

    Parameters
    ----------
    seq_dir : str
        Path to the sequence directory containing IMU data.
    calib_dir : str
        Path to the calibration directory (used to build IMU sensor params for timestamp lookup).

    Returns
    -------
    imu_data : list of dict
        Each dict contains 'timestamp', 'accel' ([x,y,z]), and 'gyro' ([x,y,z]).
    """
    imu_params = get_imu_params(calib_dir)
    imu_readings = get_imu(seq_dir)
    imu_timestamps = get_timestamps(seq_dir, imu_params)

    imu_data = []
    for ts, reading in zip(imu_timestamps, imu_readings):
        imu_data.append({
            'timestamp': ts,
            'accel': reading['accel'],
            'gyro': reading['gyro'],
        })

    return imu_data
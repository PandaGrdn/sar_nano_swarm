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
from filtering import *

# interpolates poses for the given timestamps
# param[in] src_poses: list of poses in the form of a dict {'position': [x,y,z], 'orientation: [x,y,z,w]'}
# param[in] src_stamps: list of timestamps for the src_poses
# praam[in] tgt_stamps: list of timestamps for which poses need to be calculated
# return tgt_poses: list of interpolated poses as 4x4 transformation matrices
# return tgt_indices: list of indices in tgt_stamps for which poses were able to be interpolate

import numpy as np
from scipy.spatial.transform import Rotation as R

def pose_delta(pose0, pose1):
    """
    Compute relative translation and rotation between two ground-truth poses.
    Assumes orientation is a quaternion in [x, y, z, w] scipy convention.
    """
    pos_delta = pose1['position'] - pose0['position']

    r0 = R.from_quat(pose0['orientation'])
    r1 = R.from_quat(pose1['orientation'])
    # rotation that takes pose0's orientation to pose1's orientation
    rot_delta = r1 * r0.inv()

    return pos_delta, rot_delta

def matrix_to_pose_dict(T):
    """Convert a 4x4 pose matrix into the {'position', 'orientation'} dict
    format expected by pose_delta."""
    position = T[:3, 3]
    orientation = R.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
    return {'position': position, 'orientation': orientation}


# param[in] seq: path to folder
def get_groundtruth_adjusted_radar(seq_dir, calib_dir):
    gt_params = get_groundtruth_params()
    gt_timestamps = get_timestamps(seq_dir, gt_params)
    gt_poses = get_groundtruth(seq_dir)
    
    all_radar_params = get_cascade_params(calib_dir)
    radar_params = all_radar_params['heatmap']
    radar_timestamps = get_timestamps(seq_dir, radar_params)
    
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

    
    return radar_timestamps, radar_gt, radar_indices

# def calc_groundvel(prev_point, next_point):
    
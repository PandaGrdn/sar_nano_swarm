from filtering import *
from collections import defaultdict
# from examples.python.EKF import VelocityEKF 
import matplotlib.pyplot as plt
import numpy as np

def get_clusters(radar_points):
    clusters = np.array(clustering(radar_points))
    cluster_dict_0 = defaultdict(list)
    for idx in range(len(clusters)):
        cluster_dict_0[int(clusters[idx])].append(radar_points[idx])
    
    # determining center points for cluster
    cluster_centers = []
    cluster_num_points = []
    for label, points in cluster_dict.items():
        if label == -1 or len(points) < radar_num_threshold:
            continue  # skip noise points (DBSCAN convention: -1 = noise)
        pts_arr = np.array(points)
        centroid = pts_arr[:, :3].mean(axis=0)  # mean of x, y, z only
        cluster_centers.append(centroid)
        cluster_num_points.append(len(points))
        
        
            

# thresholds
radar_num_threshold = 7
# ekf = VelocityEKF(measurement_noise=0.3)

# match timestamps to radar points
radar_map = build_radar_frame_map()
radar_dict = {frame: pts for frame, pts in radar_map}

keys = list(radar_dict.keys())

for k0, k1 in zip(keys, keys[1:]):
    radar_points_0 = radar_dict[k0]
    radar_points_1 = radar_dict[k1]

    # extract x/y columns across ALL points, not individual points
    x0 = [p[0] for p in radar_points_0]
    y0 = [p[1] for p in radar_points_0]
    z0 = np.asarray([p[2] for p in radar_points_0])
    x1 = [p[0] for p in radar_points_1]
    y1 = [p[1] for p in radar_points_1]
    z1 = np.asarray([p[2] for p in radar_points_0])

    timestamp_0 = radar_points_0[0][5]
    timestamp_1 = radar_points_1[0][5]

    alpha0 = np.where(z0 > 1, np.clip(z0, 0, 1), 0)
    alpha1 = np.where(z1 > 1, np.clip(z1, 0, 1), 0)
    
    z_min, z_max = alpha0.min(), alpha0.max()
    if z_max > z_min:
        alpha0_norm = (alpha0 - z_min) / (z_max - z_min)
    else:
        alpha0_norm = np.zeros_like(alpha0)
    
    z_min, z_max = alpha1.min(), alpha1.max()
    if z_max > z_min:
        alpha1_norm = (alpha1 - z_min) / (z_max - z_min)
    else:
        alpha1_norm = np.zeros_like(alpha1)
        

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    
    
    
    if frame_idx != 0:
        matched_prev, matched_curr, match_weights = [], [], []
        new_clusters = []
    
        for i, center in enumerate(cluster_centers):
            j, dist = best_match(center, prev_cluster_centers)
            new_clusters.append((center, cluster_num_points[i]))
            if j is not None:
                matched_prev.append(prev_cluster_centers[j])
                matched_curr.append(center)
                # weight: more points in cluster = better centroid, closer range = less noise
                r = np.linalg.norm(prev_cluster_centers[j])
                match_weights.append(cluster_num_points[i] / (r**2 + 1e-3))
    
        print("new clusters detected: ", len(new_clusters))
        for center, num_points in new_clusters:
            print(f"  new cluster at {center} with {num_points} points")
    
        if len(matched_prev) >= 1:
            dt = timestamp - prev_timestamp
            vels = []
            omegas = []
            for p, c in zip(matched_prev, matched_curr):
                v, omega = estimate_ego_motion(p, c, dt)
                vels.append(v)
                omegas.append(omega)
    
            weights = np.array(match_weights)
            weights /= weights.sum()
    
            # single fused velocity measurement, weighted by confidence
            fused_landmark_vel = np.average(vels, axis=0, weights=weights)
    
            # optional: residual spread as a proxy for measurement confidence this frame
            residuals = np.array(vels) - fused_landmark_vel
            measurement_var = np.average(np.sum(residuals**2, axis=1), weights=weights)
    
            for i in range(len(vels)):
                print("cluster #", i, "| v: ", vels[i], "| omega: ", omegas[i])
            print(f"  fused landmark velocity: {fused_landmark_vel}, "
                f"residual var: {measurement_var:.4f}")
            ekf.update(fused_landmark_vel)  # one update, not N
    
        print(f"  EKF fused velocity: {ekf.get_velocity()}, bias: {ekf.get_bias()}")  

    axes[0].scatter(x0, y0, c='r', s=4, alpha=alpha0_norm)
    axes[0].set_title(f"Frame: {k0} || Timestamp: {timestamp_0}")

    axes[1].scatter(x1, y1, c='b', s=4, alpha=alpha1_norm)
    axes[1].set_title(f"Frame: {k1} || Timestamp: {timestamp_1}")

    plt.show()
    plt.close(fig)
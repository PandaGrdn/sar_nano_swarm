from filtering import *
from collections import defaultdict
from ekf import VelocityEKF  # or paste the class directly into this file
import numpy as np

# thresholds
radar_num_threshold = 7
ekf = VelocityEKF(measurement_noise=0.3)

# match timestamps to radar points
radar_map = build_radar_frame_map()
radar_dict = {frame: pts for frame, pts in radar_map}

prev_cluster_centers = []
prev_cluster_num_points = []
prev_timestamp = -1

for frame_idx in list(radar_dict):
    print("frame #: ", frame_idx, "    points: ", len(radar_dict[frame_idx]))
    radar_points = radar_dict[frame_idx]
    timestamp = radar_points[0][5]

    # cluster the radar points per each frame to determine potential landmarks
    clusters = np.array(clustering(radar_points))
    cluster_dict = defaultdict(list)
    for idx in range(len(clusters)):
        cluster_dict[int(clusters[idx])].append(radar_points[idx])

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

    prev_timestamp = timestamp
    prev_cluster_centers = cluster_centers
    prev_cluster_num_points = cluster_num_points

    print({k: len(v) for k, v in cluster_dict.items()})
    print("cluster centers (x, y, z):")
    for i, center in enumerate(cluster_centers):
        print(f"  cluster {i}: {center}")

    x = input("_________________")
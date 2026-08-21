from filtering import *
from collections import defaultdict
from velocity import VelocityEKF  # or paste the class directly into this file
import numpy as np

# thresholds
radar_num_threshold = 7

# match timestamps to radar points
radar_map = build_radar_frame_map()
radar_dict = {frame: pts for frame, pts in radar_map}

# load IMU data once, sorted by timestamp
imu_data = get_IMU_data()
imu_data.sort(key=lambda r: r['timestamp'])
imu_idx = 0  # pointer into imu_data, advances monotonically alongside radar frames

# initialize EKF
ekf = VelocityEKF(measurement_noise=0.3)
prev_imu_ts = None

prev_cluster_centers = []
prev_cluster_num_points = []
prev_timestamp = -1
print(list(radar_dict))
for frame_idx in list(radar_dict):
    print("frame #: ", frame_idx, "    points: ", len(radar_dict[frame_idx]))
    radar_points = radar_dict[frame_idx]
    timestamp = radar_points[0][5]

    # --- NEW: run EKF prediction using all IMU samples up to this radar timestamp ---
    while imu_idx < len(imu_data) and imu_data[imu_idx]['timestamp'] <= timestamp:
        reading = imu_data[imu_idx]
        accel = np.array(reading['accel'])
        ts = reading['timestamp']
        if prev_imu_ts is not None:
            dt = ts - prev_imu_ts
            if dt > 0:
                ekf.predict(accel, dt)
        prev_imu_ts = ts
        imu_idx += 1

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
        # compare with previous frame's cluster centers to determine if any are new
        new_clusters = []
        vel_approx = []
        for i, center in enumerate(cluster_centers):
            for j, prev_center in enumerate(prev_cluster_centers):
                cov = np.diag([0.5, 0.05, 0.05])  # larger variance = less penalty for being off in that axis
                dist_weighted = log_likelihood_distance(prev_center, center, covariance=cov)
                if dist_weighted < 1.0:  # threshold distance to consider same cluster
                    vel_approx.append(estimate_velocity(prev_center, center, prev_timestamp, timestamp))
                    break
            new_clusters.append((center, cluster_num_points[i]))

        print("new clusters detected: ", len(new_clusters))
        for center, num_points in new_clusters:
            print(f"  new cluster at {center} with {num_points} points")
        for vel_vector, speed in vel_approx:
            print(f"  estimated velocity: {vel_vector} with speed {speed:.2f} m/s")
            # --- NEW: feed each landmark-derived velocity into the EKF as a measurement ---
            ekf.update(vel_vector)

        # --- NEW: report the EKF's fused velocity estimate for this frame ---
        print(f"  EKF fused velocity: {ekf.get_velocity()}, bias: {ekf.get_bias()}")

    prev_timestamp = timestamp
    prev_cluster_centers = cluster_centers
    prev_cluster_num_points = cluster_num_points

    print({k: len(v) for k, v in cluster_dict.items()})
    print("cluster centers (x, y, z):")
    for i, center in enumerate(cluster_centers):
        print(f"  cluster {i}: {center}")

    x = input("_________________")
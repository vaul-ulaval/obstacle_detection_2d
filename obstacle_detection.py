import time
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from sklearn.cluster import DBSCAN

from load_data import Map, Pose, Scan, load_map, load_poses, load_scans


def fit_circle_least_squares(points):
    # Need at least 3 points to fit a circle
    if len(points) < 3:
        return None, None, np.inf
    
    x = points[:, 0]
    y = points[:, 1]
    
    # Circle equation: (x - cx)^2 + (y - cy)^2 = r^2
    # Expanded: x^2 + y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)
    A = np.column_stack([2*x, 2*y, np.ones_like(x)])
    b = x**2 + y**2
    
    try:
        params, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        cx, cy, c = params
        radius = np.sqrt(c + cx**2 + cy**2)
        
        distances = np.sqrt((x - cx)**2 + (y - cy)**2)
        errors = np.abs(distances - radius)
        mean_error = np.mean(errors)
        
        return (cx, cy), radius, mean_error
    except np.linalg.LinAlgError:
        return None, None, np.inf


def classify_obstacle(points, max_radius=0.5, max_error=0.05):
    center, radius, error = fit_circle_least_squares(points)
    
    if center is None:
        return 'wall', None
    
    # Check if the fit is good enough and radius is reasonable
    if error < max_error and radius < max_radius:
        return 'circle', (center, radius)
    else:
        return 'wall', None


def obstacle_detection(map: Map, pose: Pose, scan: Scan):
    points = scan_to_points(scan)

    
    start_time = time.perf_counter()
    # Rule of thumb for min_samples: nb_dimensions * 2 (See https://dl.acm.org/doi/10.1145/3068335)
    # Higher eps = larger clusters = more compute time
    clustering = DBSCAN(eps=0.5, min_samples=4).fit(points)
    end_time = time.perf_counter()
    
    dbscan_time = end_time - start_time
    print(f"DBSCAN execution time: {dbscan_time*1000:.2f} ms")

    labels = clustering.labels_
    
    # Classify each cluster as circle or wall
    obstacles = []
    unique_labels = set(labels)
    if -1 in unique_labels:
        unique_labels.remove(-1)  # Remove noise label
    
    for label in unique_labels:
        cluster_points = points[labels == label]
        obstacle_type, params = classify_obstacle(cluster_points)
        obstacles.append({
            'label': label,
            'type': obstacle_type,
            'params': params,
            'points': cluster_points
        })
        
        if obstacle_type == 'circle':
            center, radius = params
            print(f"Cluster {label}: CIRCLE - center=({center[0]:.2f}, {center[1]:.2f}), radius={radius:.2f}m")
        else:
            print(f"Cluster {label}: WALL - {len(cluster_points)} points")
    
    # Plot the clustering results with circle fits
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Get unique labels (excluding noise points labeled as -1)
    colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))
    
    # Plot noise points first
    if -1 in labels:
        noise_mask = (labels == -1)
        noise_points = points[noise_mask]
        ax.scatter(noise_points[:, 0], noise_points[:, 1], c='black', s=10, 
                   label='Noise', alpha=0.3, marker='x')
    
    # Plot each obstacle
    for obstacle, color in zip(obstacles, colors):
        label = obstacle['label']
        obs_points = obstacle['points']
        obstacle_type = obstacle['type']
        
        if obstacle_type == 'circle':
            marker_size = 30
            center, radius = obstacle['params']
            # Draw the fitted circle
            circle_patch = Circle(center, radius, fill=False, edgecolor=color, 
                                  linewidth=2, linestyle='--', label=f'Cluster {label} (Circle)')
            ax.add_patch(circle_patch)
            # Mark the center
            ax.scatter(center[0], center[1], c=[color], s=100, marker='+', linewidths=3)
        else:
            marker_size = 30
            label_text = f'Cluster {label} (Wall)'
        
        # Plot the actual points
        ax.scatter(obs_points[:, 0], obs_points[:, 1], c=[color], s=marker_size, 
                   alpha=0.6, edgecolors='k', linewidths=0.5)
    
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title(f'Obstacle Detection: Circles vs Walls\n(DBSCAN time={dbscan_time*1000:.2f}ms)')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()

    return obstacles

def scan_to_points(scan: Scan):
    angles = np.arange(scan.metadata.angle_min, scan.metadata.angle_max, scan.metadata.angle_increment)
    ranges = scan.ranges

    return np.array([
        ranges * np.cos(angles),
        ranges * np.sin(angles)
    ]).T


if __name__ == "__main__":
    from viz import draw_scene

    data_folder = Path("blitz_obstacle_detection_extracted")
    map = load_map(data_folder)
    poses = load_poses(data_folder)
    scans = load_scans(data_folder)

    i = 1400
    pose = poses[i]
    scan = scans[i]

    obstacles = obstacle_detection(map, pose, scan)
        
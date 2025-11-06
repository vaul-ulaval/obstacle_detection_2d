from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt
from load_data import Map, Pose, Scan, ScanMetadata, load_map, load_poses, load_scans
from scipy.ndimage import binary_dilation


def obstacle_detection(map: Map, pose: Pose, scan: Scan):
    # TODO: Implement obstacle detection algorithm
    MAX_RADIUS = 0.2

    ranges = np.array(scan.ranges, dtype=float)
    angles = scan.metadata.angle_min + np.arange(ranges.size) * scan.metadata.angle_increment

    x_r = ranges * np.cos(angles) # (N,)
    y_r = ranges * np.sin(angles) # (N,)
    robot_coords = np.stack((x_r, y_r), axis=1) # (N,2)

    cos_yaw = np.cos(pose.yaw)
    sin_yaw = np.sin(pose.yaw)
    rotation = np.array([[cos_yaw, -sin_yaw],
                         [sin_yaw, cos_yaw]]) # (2,2)
    reference_coords = robot_coords @ rotation.T # (N,2)
    reference_coords += np.array([pose.x, pose.y])

    diffs = np.diff(reference_coords, axis=0)
    r = np.linalg.norm(diffs, axis=1)
    breaks = np.where(r > MAX_RADIUS)[0] + 1
    splits = np.split(reference_coords, breaks)
    obstacles = [group.tolist() for group in splits if len(group) > 0]
            
    obstacles = cluster_obstacles(obstacles)

    obstacles = remove_walls(obstacles, map)

    return obstacles

def cluster_obstacles(obstacles, min_length = 0.5):
    clustered_obstacles = []

    for i, obs in enumerate(obstacles):
        if len(obs) == 0:
            continue
        coords_1 = np.array([obs[0], obs[(len(obs) - 1)//2], obs[-1]])

        for j, obs2 in enumerate(obstacles):
            if j <= i or len(obs2) == 0:
                continue

            coords_2 = np.array([obs2[0], obs2[len(obs2)//2], obs2[-1]])

            diffs = coords_1[:, None, :] - coords_2[None, :, :]

            dists = np.linalg.norm(diffs, axis=2)
            if np.any(dists < min_length):
                obs.extend(obs2)
                obstacles[j] = []

        clustered_obstacles.append(obs)

    return clustered_obstacles

def world_to_map_indices(map: Map, x: float, y: float):
    col_f = (x - map.metadata.origin_x) / map.metadata.resolution
    row_f = (y - map.metadata.origin_y) / map.metadata.resolution

    col = int(np.floor(col_f))
    row = int(np.floor(row_f))

    col = int(np.clip(col, 0, map.metadata.width - 1))
    row = int(np.clip(row, 0, map.metadata.height - 1))

    return row, col

def remove_walls(obstacles, map: Map, wall_thickness = 10):

    walls = (map.grid == 100)

    structure = np.ones((wall_thickness, wall_thickness))

    dilated = binary_dilation(walls, structure=structure)

    thickened_map = np.where(dilated, 100, 0)

    for obs in obstacles:
        if len(obs) == 0:
            continue

        for coords in obs:
            x, y = coords[0], coords[1]
            row, col = world_to_map_indices(map, x, y)

            if thickened_map[row, col] == 100:
                obs.clear()
                break

    return obstacles

if __name__ == "__main__":
    from viz import draw_scene

    data_folder = Path("blitz_obstacle_detection_extracted")
    map = load_map(data_folder)
    poses = load_poses(data_folder)
    scans = load_scans(data_folder)

    i = 1400
    pose = poses[i]
    scan = scans[i]

    fig, ax = plt.subplots()

    obstacles = obstacle_detection(map, pose, scan)
    draw_scene(map, pose, scan, obstacles, ax)

    plt.show()
    
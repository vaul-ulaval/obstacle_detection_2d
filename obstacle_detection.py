from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt
from load_data import Map, Pose, Scan, ScanMetadata, load_map, load_poses, load_scans


def obstacle_detection(map: Map, pose: Pose, scan: Scan):
    # TODO: Implement obstacle detection algorithm
    MAX_RADIUS = 0.2

    metadata = scan.metadata

    obstacles = [[]]

    scan.ranges

    ranges_in_robot_coordinates = [(
                            distance * np.cos(metadata.angle_min + i * metadata.angle_increment),
                            distance * np.sin(metadata.angle_min + i * metadata.angle_increment) 
                            ) for i, distance in enumerate(scan.ranges)]

    range_in_referance_plan = [(
                            coord[0]*np.cos(pose.yaw) - coord[1]*np.sin(pose.yaw) + pose.x,
                            coord[0]*np.sin(pose.yaw) + coord[1]*np.cos(pose.yaw) + pose.y
                            ) for coord in ranges_in_robot_coordinates]

    for i, coord in enumerate(range_in_referance_plan):
        if i == 0:
            obstacles[0].append(coord)
            continue

        x_0 = range_in_referance_plan[i - 1][0]
        y_0 = range_in_referance_plan[i - 1][1]

        x_1 = coord[0]
        y_1 = coord[1]

        r = np.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)

        if (r <= MAX_RADIUS):
            obstacles[len(obstacles) - 1].append(coord)
        else:
            obstacles.append([coord])
            
    obstacles = cluster_obstacles(obstacles)

    obstacles = remove_walls(obstacles, map)

    return obstacles

def cluster_obstacles(obstacles, min_length = 0.5):
    clustered_obstacles = []

    for i, obs in enumerate(obstacles):
        if len(obs) == 0:
            continue
        coords_1 = [obs[0], obs[(len(obs) - 1)//2], obs[-1]]

        for j, obs2 in enumerate(obstacles):
            if j <= i or len(obs2) == 0:
                continue

            coords_2 = [obs2[0], obs2[len(obs2)//2], obs2[-1]]
            
            dists = [np.sqrt((c1[0] - c2[0])**2 + (c1[1] - c2[1])**2) for c1 in coords_1 for c2 in coords_2]

            if any(dist < min_length for dist in dists):
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

def remove_walls(obstacles, map: Map, enlarge_wall = 4):
    for obs in obstacles:
        if len(obs) == 0:
            continue
        
        for coords in obs:
            x, y = coords[0], coords[1]
            row, col = world_to_map_indices(map, x, y)

            if map.grid[row, col] == 100:
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
    
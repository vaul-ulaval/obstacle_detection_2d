from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt
from load_data import Map, Pose, Scan, ScanMetadata, load_map, load_poses, load_scans


def obstacle_detection(map: Map, pose: Pose, scan: Scan):
    # TODO: Implement obstacle detection algorithm
    MAX_DX = 0.5  # meters
    MAX_DY = 0.5  # meters


    obstacles = [[]]

    for i, scan_range in scan.ranges:
        x, y = scan_range_to_coordinates(pose, scan_range, scan.metadata, i)
        if len(obstacles[len(obstacles) - 1]) == 0:
            obstacles.append([(x, y)])
            continue

        last_x, last_y = obstacles[len(obstacles) - 1][-1]
        dx = x - last_x
        dy = y - last_y

        if dx > MAX_DX or dy > MAX_DY:
            obstacles.append([(x, y)])

    return obstacles

def scan_range_to_coordinates(pose: Pose, range: float, metadata: ScanMetadata, index: int) -> (float, float):
    angle = metadata.angle_min + index * metadata.angle_increment
    x_lidar = range * np.cos(angle)
    y_lidar = range * np.sin(angle)

    x = x_lidar*np.cos(pose.yaw) - y_lidar*np.sin(pose.yaw) + pose.x
    y = x_lidar*np.sin(pose.yaw) + y_lidar*np.cos(pose.yaw) + pose.y

    return x, y

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
    
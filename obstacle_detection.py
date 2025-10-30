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

    range_in_robot_coordinates = [(
                            distance * np.cos(metadata.angle_min + i * metadata.angle_increment),
                            distance * np.sin(metadata.angle_min + i * metadata.angle_increment) 
                            ) for i, distance in enumerate(scan.ranges)]

    range_in_referance_plan = [(
                            coords[0]*np.cos(pose.yaw) - coords[1]*np.sin(pose.yaw) + pose.x,
                            coords[0]*np.sin(pose.yaw) + coords[1]*np.sin(pose.yaw) + pose.y
                            ) for coords in range_in_robot_coordinates]



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
    
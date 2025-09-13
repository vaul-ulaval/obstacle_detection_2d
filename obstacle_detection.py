from pathlib import Path

from matplotlib import pyplot as plt
from load_data import Map, Pose, Scan, load_map, load_poses, load_scans


def obstacle_detection(map: Map, pose: Pose, scan: Scan):
    # TODO: Implement obstacle detection algorithm
    return []


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
    
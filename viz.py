from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from load_data import Map, Pose, Scan, load_map, load_poses, load_scans
from scipy.spatial.transform import Rotation as R

from obstacle_detection import draw_obstacles, obstacle_detection

def draw_map(map: Map, ax):
    width, height = map.metadata.width, map.metadata.height
    width = int(width)
    height = int(height)

    grid = map.grid

    mapped_grid = np.zeros_like(grid, dtype=float)

    mapped_grid[grid == -1] = 0.5

    mask = (grid >= 0) & (grid <= 100)
    mapped_grid[mask] = 1.0 - (grid[mask] / 100.0)

    ax.imshow(mapped_grid, cmap="gray", origin="lower")
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_aspect('equal')

def draw_pose(pose, map: Map, ax):
    origin_x, origin_y = map.metadata.origin_x, map.metadata.origin_y
    resolution = map.metadata.resolution
    
    T_map_to_img = np.array([
        [1/resolution, 0, -origin_x/resolution],
        [0, 1/resolution, -origin_y/resolution],
        [0, 0, 1]
    ])

    p = np.array([pose.x, pose.y, 1])
    p_map = T_map_to_img @ p
    
    ax.scatter(p_map[0], p_map[1], s=10, c="red")
    arrow_length = 10
    arrow_dx = arrow_length * np.cos(pose.yaw)
    arrow_dy = arrow_length * np.sin(pose.yaw)
    ax.arrow(p_map[0], p_map[1], arrow_dx, arrow_dy, head_width=3, head_length=3, fc="red", ec="red")

def draw_scan(scan: Scan, pose: Pose, map: Map, ax):
    origin_x, origin_y = map.metadata.origin_x, map.metadata.origin_y
    resolution = map.metadata.resolution

    T_map_to_img = np.array([
        [1/resolution, 0, -origin_x/resolution],
        [0, 1/resolution, -origin_y/resolution],
        [0, 0, 1]
    ])
    
    rot = R.from_euler('z', pose.yaw)
    T_map_to_base_link = np.eye(3)
    T_map_to_base_link[:2, :2] = rot.as_matrix()[:2, :2]
    T_map_to_base_link[:2, 2] = [pose.x, pose.y]

    T_laser_to_base_link = np.eye(3)
    T_laser_to_base_link[:2, 2] = [0.109, 0.0]

    ranges = np.array(scan.ranges)
    angles = scan.metadata.angle_min + np.arange(len(ranges)) * scan.metadata.angle_increment
    xs_robot = ranges * np.cos(angles)
    ys_robot = ranges * np.sin(angles)
    if scan.metadata.range_max > 0:
        mask = ranges < scan.metadata.range_max
        xs_robot = xs_robot[mask]
        ys_robot = ys_robot[mask]

    points_laser = np.stack([xs_robot, ys_robot, np.ones_like(xs_robot)], axis=0)

    points_img = T_map_to_img @ T_map_to_base_link @ T_laser_to_base_link @ points_laser

    ax.scatter(points_img[0, :], points_img[1, :], s=1, c="blue")


if __name__ == "__main__":
    data_folder = Path("blitz_obstacle_detection_extracted")
    map = load_map(data_folder)
    poses = load_poses(data_folder)
    scans = load_scans(data_folder)

    fig, ax = plt.subplots()
    draw_map(map, ax)

    display_interval = 20
    frames_to_display = range(0, len(poses), display_interval)

    def update(frame_idx):
        ax.clear()
        draw_map(map, ax)
        pose = poses[frame_idx]
        scan = scans[frame_idx]

        draw_pose(pose, map, ax)
        draw_scan(scan, pose, map, ax)

        obstacles = obstacle_detection(map, pose, scan)
        draw_obstacles(obstacles, ax)

        ax.set_title(f"Robot playback")

    anim = FuncAnimation(fig, update, frames=frames_to_display, interval=200)  # interval in ms is still fine
    plt.show()
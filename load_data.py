from dataclasses import dataclass
from os import times
from pathlib import Path
import time

import numpy as np


@dataclass
class MapMetadata:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_z: float

@dataclass
class Map:
    metadata: MapMetadata
    grid: np.ndarray # 2D np array where -1 = unknown, 0 = free, 100 = occupied

@dataclass
class Pose:
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

@dataclass
class ScanMetadata:
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    range_min: float
    range_max: float

@dataclass
class Scan:
    timestamp: float
    metadata: ScanMetadata
    ranges: np.ndarray

def load_map(data_folder) -> Map:
    map_data = np.loadtxt(data_folder / "map_data.csv", delimiter=",")
    map_metadata = np.loadtxt(data_folder / "map_metadata.csv", delimiter=",", skiprows=1)

    width, height, resolution, x, y, z = map_metadata
    width = int(width)
    height = int(height)

    return Map(
        grid=map_data.reshape((height, width)),
        metadata=MapMetadata(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=x,
            origin_y=y,
            origin_z=z
        ) 
    )

def load_poses(data_folder):
    poses_csv = np.loadtxt(data_folder / "poses.csv", delimiter=",", skiprows=1)

    poses = [
        Pose(
            timestamp=row[0],
            x=row[1],
            y=row[2],
            z=row[3],
            roll=row[4],
            pitch=row[5],
            yaw=row[6]
        )
        for row in poses_csv
    ]

    return poses

def load_scans(data_folder):
    scans_csv = np.loadtxt(data_folder / "scan.csv", delimiter=",", skiprows=1)

    scans = [
        Scan(
            timestamp=row[0],
            metadata=ScanMetadata(
                angle_min=row[1],
                angle_max=row[2],
                angle_increment=row[3],
                time_increment=row[4],
                range_min=row[5],
                range_max=row[6]
            ),
            ranges=np.array(row[7:])
        )
        for row in scans_csv
    ]

    return scans

if __name__ == "__main__":
    data_folder = Path("blitz_obstacle_detection_extracted")
    map = load_map(data_folder)
    poses = load_poses(data_folder)
    scans = load_scans(data_folder)

    print(f"Map: {map.metadata.width}x{map.metadata.height} at {map.metadata.resolution} m/px")
    print(f"Loaded {len(poses)} poses")
    print(f"Loaded {len(scans)} scans")
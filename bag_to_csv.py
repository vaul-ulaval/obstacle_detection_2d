from pathlib import Path
from time import time_ns
from scipy.spatial.transform import Rotation as R

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from tqdm import tqdm


def extract_map_data(bag_file, topic_name, output_folder):
    print(f"Extracting map data from {bag_file} on topic {topic_name} to {output_folder}...")
    first_ts = None

    with AnyReader([Path(bag_file)]) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]
        for connection, _, rawdata in tqdm(reader.messages(connections=connections)):
            msg = reader.deserialize(rawdata, connection.msgtype)

            timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

            if first_ts is None:
                first_ts = timestamp

            elapsed_time = (timestamp - first_ts) / 1e9

            if elapsed_time > 30 and elapsed_time < 31:
                data = msg.data
                width = msg.info.width
                height = msg.info.height
                resolution = msg.info.resolution
                origin = msg.info.origin.position
                x, y, z = origin.x, origin.y, origin.z

                np.savetxt(
                    output_folder / "map_data.csv",
                    data.reshape((height, width)),
                    delimiter=",",
                    comments="",
                    fmt="%.3f"
                )
                np.savetxt(
                    output_folder / "map_metadata.csv", 
                    np.array([[width, height, resolution, x, y, z]]), 
                    delimiter=',', 
                    header="width,height,resolution,x,y,z",
                    comments="",
                    fmt="%.3f"
                )


def extract_pose_data(bag_file, topic_name, output_folder):
    print(f"Extracting pose data from {bag_file} on topic {topic_name} to {output_folder}...")

    with AnyReader([Path(bag_file)]) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]

        data = []

        for connection, _, rawdata in tqdm(reader.messages(connections=connections)):
            msg = reader.deserialize(rawdata, connection.msgtype)

            timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

            x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
            qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

            r = R.from_quat([qx, qy, qz, qw])
            roll, pitch, yaw = r.as_euler('xyz', degrees=False)

            data.append([timestamp, x, y, z, roll, pitch, yaw])

    np.savetxt(
        output_folder / "poses.csv",
        data,
        delimiter=",",
        header="timestamp,x,y,z,roll,pitch,yaw",
        comments="",
        fmt="%.3f"
    )


def extract_scan_data(bag_file, topic_name, output_folder):
    print(f"Extracting scan data from {bag_file} on topic {topic_name} to {output_folder}...")

    with AnyReader([Path(bag_file)]) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]

        data = []

        for connection, _, rawdata in tqdm(reader.messages(connections=connections)):
            msg = reader.deserialize(rawdata, connection.msgtype)

            timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment
            time_increment = msg.time_increment
            range_min = msg.range_min
            range_max = msg.range_max
            ranges = msg.ranges

            data.append([timestamp, angle_min, angle_max, angle_increment, time_increment, range_min, range_max] + list(ranges))

    header = "timestamp,angle_min,angle_max,angle_increment,time_increment,range_min,range_max," + \
            ",".join([f"range_{i}" for i in range(len(ranges))])

    np.savetxt(
        output_folder / "scan.csv",
        data,
        delimiter=",",
        header=header,
        comments="",
        fmt="%.6f"
    )

if __name__ == "__main__":
    input_bag = "blitz_obstacle_detection"
    output_folder = Path("blitz_obstacle_detection_extracted")
    output_folder.mkdir(exist_ok=True)

    extract_map_data(input_bag, "/map", output_folder)
    extract_pose_data(input_bag, "/odometry/filtered", output_folder)
    extract_scan_data(input_bag, "/scan", output_folder)
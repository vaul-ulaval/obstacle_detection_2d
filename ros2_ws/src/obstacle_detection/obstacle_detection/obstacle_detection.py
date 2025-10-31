import rclpy
import message_filters
import numpy as np
from scipy.ndimage import binary_dilation
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.scan_subscription = message_filters.Subscriber(
            self,
            LaserScan,
            '/scan'
            )

        self.odometry_subscription = message_filters.Subscriber(
            self,
            Odometry,
            '/odometry/filtered'
        )

        ts = message_filters.ApproximateTimeSynchronizer([self.scan_subscription, self.odometry_subscription], 10, slop=0.1)
        ts.registerCallback(self.obstacle_detection_callback)

    def map_callback(self, msg):
        self.map = msg.data
        self.map_metadata = msg.info

    def obstacle_detection_callback(self, scan_msg, odom_msg):
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        pose = odom_msg.pose.pose
        scan_ranges = scan_msg.ranges

        obstacles = [[]]

        ranges_in_robot_coordinates = [(
                            distance * np.cos(angle_min + i * angle_increment),
                            distance * np.sin(angle_min + i * angle_increment) 
                            ) for i, distance in enumerate(scan_ranges)]

        ranges_in_referance_plan = [(
                            coord[0]*np.cos(pose.yaw) - coord[1]*np.sin(pose.yaw) + pose.x,
                            coord[0]*np.sin(pose.yaw) + coord[1]*np.cos(pose.yaw) + pose.y
                            ) for coord in ranges_in_robot_coordinates]
        
        obstacles = self.detect_obstacles(ranges_in_referance_plan)

        obstacles = self.cluster_obstacles(obstacles)

        obstacles = self.remove_walls(obstacles)

        return obstacles
    
    def detect_obstacles(self, ranges, max_distance=0.2):
        obstacles = [[]]

        for i, coord in enumerate(ranges):
            if i == 0:
                obstacles[0].append(coord)
                continue

            x_0 = ranges[i - 1][0]
            y_0 = ranges[i - 1][1]

            x_1 = coord[0]
            y_1 = coord[1]

            d = np.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)

            if (d <= max_distance):
                obstacles[-1].append(coord)
            else:
                obstacles.append([coord])

        return obstacles

    def cluster_obstacles(self, obstacles, min_length=0.5):
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

    def coordinates_to_map_indices(self, x: float, y: float):
        col_f = (x - self.map_metadata.origin_x) / self.map_metadata.resolution
        row_f = (y - self.map_metadata.origin_y) / self.map_metadata.resolution

        col = int(np.floor(col_f))
        row = int(np.floor(row_f))

        col = int(np.clip(col, 0, self.map_metadata.width - 1))
        row = int(np.clip(row, 0, self.map_metadata.height - 1))

        return row, col

    def remove_walls(self, obstacles, wall_thickness=10):
        for obs in obstacles:
            if len(obs) == 0:
                continue

            walls = (self.map == 100) | (self.map == 1)
    
            structure = np.ones((wall_thickness, wall_thickness))
    
            dilated = binary_dilation(walls, structure=structure)
    
            thickened_map = np.where(dilated, 100, 0)
    
            for coords in obs:
                x, y = coords[0], coords[1]
                row, col = self.coordinates_to_map_indices(map, x, y)
    
                if thickened_map[row, col] == 100:
                    obs.clear()
                    break
                
        return obstacles

def main(args=None):
    rclpy.init(args=args)

    obstacle_detection = ObstacleDetection()

    rclpy.spin(obstacle_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
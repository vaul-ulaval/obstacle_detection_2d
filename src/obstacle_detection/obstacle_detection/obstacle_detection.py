import rclpy
import message_filters
import numpy as np
from scipy.ndimage import binary_dilation
from rclpy.node import Node
from transforms3d.euler import quat2euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__("obstacle_detection")

        MAP_QOS = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, MAP_QOS
        )

        self.scan_subscription = message_filters.Subscriber(self, LaserScan, "/scan")

        self.odometry_subscription = message_filters.Subscriber(
            self, Odometry, "/odometry/filtered"
        )

        self.markers_pub = self.create_publisher(MarkerArray, "/obstacles_markers", 10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_subscription, self.odometry_subscription],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.obstacle_detection_callback)

    def map_callback(self, msg: OccupancyGrid):
        self.map_metadata = msg.info
        self.map = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def obstacle_detection_callback(self, scan_msg: LaserScan, odom_msg: Odometry):
        if not hasattr(self, "map"):
            return
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        pose = odom_msg.pose.pose
        ranges = np.array(scan_msg.ranges, dtype=float)
        angles = angle_min + np.arange(ranges.size) * angle_increment

        x_r = ranges * np.cos(angles)  # (N,)
        y_r = ranges * np.sin(angles)  # (N,)
        robot_coords = np.stack((x_r, y_r), axis=1)  # (N,2)

        yaw = quat2euler(
            [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ]
        )[2]
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])  # (2,2)
        reference_coords = robot_coords @ rotation.T  # (N,2)
        reference_coords += np.array([pose.position.x, pose.position.y])

        obstacles = self.detect_obstacles(reference_coords)

        obstacles = self.cluster_obstacles(obstacles)

        obstacles = self.remove_walls(obstacles)

        obstacles = self.clean_obstacles(obstacles)

        self.publish_markers(obstacles)
        
        return obstacles

    def detect_obstacles(self, reference_coords, max_distance=0.5):
        obstacles = [[]]

        diffs = np.diff(reference_coords, axis=0)
        r = np.linalg.norm(diffs, axis=1)
        breaks = np.where(r > max_distance)[0] + 1
        splits = np.split(reference_coords, breaks)
        obstacles = [group.tolist() for group in splits if len(group) > 0]

        return obstacles

    def cluster_obstacles(self, obstacles, min_length=0.5):
        clustered_obstacles = []

        for i, obs in enumerate(obstacles):
            if len(obs) == 0:
                continue
            coords_1 = np.array([obs[0], obs[(len(obs) - 1) // 2], obs[-1]])

            for j, obs2 in enumerate(obstacles):
                if j <= i or len(obs2) == 0:
                    continue

                coords_2 = np.array([obs2[0], obs2[len(obs2) // 2], obs2[-1]])

                diffs = coords_1[:, None, :] - coords_2[None, :, :]

                dists = np.linalg.norm(diffs, axis=2)
                if np.any(dists < min_length):
                    obs.extend(obs2)
                    obstacles[j] = []

            clustered_obstacles.append(obs)

        return clustered_obstacles

    def world_to_map_indices(self, x: float, y: float):
        col_f = (x - self.map_metadata.origin.position.x) / self.map_metadata.resolution
        row_f = (y - self.map_metadata.origin.position.y) / self.map_metadata.resolution

        col = int(np.floor(col_f))
        row = int(np.floor(row_f))

        col = int(np.clip(col, 0, self.map_metadata.width - 1))
        row = int(np.clip(row, 0, self.map_metadata.height - 1))

        return row, col

    def is_wall(self, coord, thickened_map):
        row, col = self.world_to_map_indices(coord[0], coord[1])

        return thickened_map[row, col] == 100

    def get_thickened_map(self, wall_thickness=10):
        walls = self.map == 100
        structure = np.ones((wall_thickness, wall_thickness))
        dilated = binary_dilation(walls, structure=structure)

        return np.where(dilated, 100, 0)

    def remove_walls(self, obstacles):
        thickened_map = self.get_thickened_map()

        for obs in obstacles:
            if len(obs) == 0:
                continue

            for coords in obs:
                x, y = coords[0], coords[1]

                if self.is_wall((x, y), thickened_map):
                    obs.clear()
                    break

        return obstacles

    def clean_obstacles(self, obstacles, min_size=3):
        for obs in obstacles:
            if len(obs) < min_size:
                obs.clear()

        return obstacles
    
    def publish_markers(self, obstacles, frame_id="map"):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear
        m_clear = Marker()
        m_clear.header.frame_id = frame_id
        m_clear.header.stamp = now
        m_clear.ns = "obstacles"
        m_clear.id = 0
        m_clear.action = Marker.DELETEALL
        arr.markers.append(m_clear)

        #  obstacles
        from geometry_msgs.msg import Point
        for k, obs in enumerate(obstacles, start=1):  # start=1
            if not obs:
                continue
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = "obstacles"
            m.id = k
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.color.a = 1.0
            m.color.r = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000
            m.points = [Point(x=float(x), y=float(y), z=0.0) for (x, y) in obs]
            arr.markers.append(m)

        self.markers_pub.publish(arr)



def main(args=None):
    rclpy.init(args=args)

    obstacle_detection = ObstacleDetection()

    rclpy.spin(obstacle_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

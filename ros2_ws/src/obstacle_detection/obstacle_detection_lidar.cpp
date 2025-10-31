#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/utils.h>

#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

class ObstaclesDetectionLidar : public rclcpp::Node {
   public:
    ObstaclesDetectionLidar() : Node("obstacles_detection_lidar") {
        map_acquired = false;
        tf_aquired = false;

        init_params();
        setup_subscribers();
        setup_publishers();

        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ObstaclesDetectionLidar::update_params, this));

        RCLCPP_INFO(get_logger(), "Node has been started.");
    }

   private:
    void init_params() {
        map_topic_ = declare_parameter("map_topic", "/map");
        scan_topic_ = declare_parameter("scan_topic", "scan");
        odom_topic_ = declare_parameter("odom_topic", "odom");
        scan_frame_ = declare_parameter("scan_frame", "laser");
        base_frame_ = declare_parameter("base_frame", "base_link");
        safety_margin_ = declare_parameter("safety_margin", 0.5);
        debug_ = declare_parameter("debug", false);
        debug_points_topic_ = declare_parameter("debug_points_topic", "obstacles/points_in_map");
        debug_map_topic_ = declare_parameter("debug_map_topic", "obstacles/expanded_map");
        debug_obstacles_topic_ = declare_parameter("debug_scan_topic", "obstacles/visualize");
        RCLCPP_INFO(get_logger(), "Parameters initialized, %s, %s", scan_frame_.c_str(), base_frame_.c_str());
    }

    void update_params() {
        double safety_margin = get_parameter("safety_margin").as_double();
        if (safety_margin != safety_margin_) {
            safety_margin_ = safety_margin;
            RCLCPP_INFO(get_logger(), "Safety margin updated to %f.", safety_margin_);
            expand_map();
        }
    }

    void setup_subscribers() {
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
            std::bind(&ObstaclesDetectionLidar::map_callback, this, std::placeholders::_1));

        if (base_frame_ == scan_frame_) {
            laser_x_ = 0.0;
            laser_y_ = 0.0;
            tf_aquired = true;
        } else {
            tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
                "/tf_static", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
                std::bind(&ObstaclesDetectionLidar::static_tf_callback, this, std::placeholders::_1));
        }

        scan_sub_.subscribe(this, scan_topic_);
        odom_sub_.subscribe(this, odom_topic_);

        ts_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>>(
            scan_sub_, odom_sub_, 10);
        ts_->registerCallback(std::bind(&ObstaclesDetectionLidar::laser_odom_callback, this, std::placeholders::_1,
                                        std::placeholders::_2));
    }

    void setup_publishers() {
        obstacles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("obstacles", 10);
        if (debug_) {
            debug_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(debug_points_topic_, 10);
            debug_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(debug_map_topic_, 10);
            debug_obstacles_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(debug_obstacles_topic_, 10);
        }
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
        map_ = map_msg;
        map_acquired = true;
        RCLCPP_INFO(get_logger(), "Map acquired.");
        expand_map();
        map_sub_.reset();
    }

    void expand_map() {
        int kernel_size = std::ceil(safety_margin_ / map_->info.resolution) * 2;
        cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_8U);
        cv::Mat map_grid = cv::Mat(map_->info.height, map_->info.width, CV_8S, &map_->data[0]);
        cv::compare(map_grid, 0, map_grid, cv::CMP_NE);
        cv::dilate(map_grid / 255, expanded_map_, kernel, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    }

    void static_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg) {
        for (const auto& tf : tf_msg->transforms) {
            if (tf.child_frame_id == scan_frame_ && tf.header.frame_id == base_frame_) {
                laser_x_ = tf.transform.translation.x;
                laser_y_ = tf.transform.translation.y;
                tf_aquired = true;
                RCLCPP_INFO(get_logger(), "Scan offset acquired, x: %f, y: %f.", laser_x_, laser_y_);
                tf_sub_.reset();
                return;
            }
        }
        RCLCPP_INFO(get_logger(), "Could not get %s to %s transform.", scan_frame_.c_str(), base_frame_.c_str());
    }

    void laser_odom_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                             const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
        if (!(map_acquired && tf_aquired)) {
            return;
        }

        double robot_x = odom_msg->pose.pose.position.x;
        double robot_y = odom_msg->pose.pose.position.y;
        double robot_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);

        std::vector<cv::Point2d> points_in_map = get_points_in_map(scan_msg, {robot_x, robot_y, robot_yaw});
        std::vector<int> obstacle_indices, free_indices;
        find_obstacles_indices(points_in_map, obstacle_indices, free_indices);

        std::vector<cv::Point2d> obstacle_points;
        obstacle_points.reserve(obstacle_indices.size());
        for (int index : obstacle_indices) {
            obstacle_points.push_back(points_in_map[index]);
        }

        publish_obstacle_points(obstacle_points, scan_msg->header.stamp);

        if (debug_) {
            publish_debug_points(points_in_map, scan_msg->header.stamp);
            publish_debug_obstacles(scan_msg, free_indices);
            publish_debug_map();
        }
    }

    std::vector<cv::Point2d> get_points_in_map(const sensor_msgs::msg::LaserScan::ConstSharedPtr laser_data,
                                               const std::vector<double>& pose) {
        std::vector<float> ranges = laser_data->ranges;
        std::vector<cv::Point2d> points;
        points.reserve(ranges.size());
        for (size_t i = 0; i < ranges.size(); ++i) {
            double angle = laser_data->angle_min + i * laser_data->angle_increment;
            double x_base_link = ranges[i] * std::cos(angle) + laser_x_;
            double y_base_link = ranges[i] * std::sin(angle) + laser_y_;
            double x_map = x_base_link * std::cos(pose[2]) - y_base_link * std::sin(pose[2]) + pose[0];
            double y_map = x_base_link * std::sin(pose[2]) + y_base_link * std::cos(pose[2]) + pose[1];
            points.emplace_back(x_map, y_map);
        }

        return points;
    }

    void find_obstacles_indices(const std::vector<cv::Point2d>& points, std::vector<int>& obstacle_indices,
                                std::vector<int>& free_indices) {
        cv::Point2d origin(map_->info.origin.position.x, map_->info.origin.position.y);
        double resolution = map_->info.resolution;
        int width = expanded_map_.cols;
        int height = expanded_map_.rows;

        obstacle_indices.clear();
        free_indices.clear();

        for (size_t i = 0; i < points.size(); ++i) {
            const auto& point = points[i];

            cv::Point2i map_index = ((point - origin) / resolution);
            map_index.x = std::max(0, std::min(map_index.x, width - 1));
            map_index.y = std::max(0, std::min(map_index.y, height - 1));

            uchar cell_value = expanded_map_.at<uchar>(map_index.y, map_index.x);

            if (cell_value == 0) {
                obstacle_indices.push_back(i);
            } else if (cell_value == 1) {
                free_indices.push_back(i);
            }
        }
    }

    void publish_debug_points(const std::vector<cv::Point2d>& points, const rclcpp::Time& stamp) {
        sensor_msgs::msg::PointCloud2 pc_message;
        pc_message.header.stamp = stamp;
        pc_message.header.frame_id = map_->header.frame_id;
        pc_message.height = 1;
        pc_message.width = points.size();
        pc_message.fields.resize(2);
        pc_message.fields[0].name = "x";
        pc_message.fields[0].offset = 0;
        pc_message.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc_message.fields[0].count = 1;
        pc_message.fields[1].name = "y";
        pc_message.fields[1].offset = 4;
        pc_message.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc_message.fields[1].count = 1;
        pc_message.point_step = 8;
        pc_message.row_step = pc_message.point_step * pc_message.width;
        pc_message.data.resize(pc_message.row_step);
        for (size_t i = 0; i < points.size(); ++i) {
            float x = static_cast<float>(points[i].x);
            float y = static_cast<float>(points[i].y);
            std::memcpy(&pc_message.data[i * pc_message.point_step], &x, sizeof(float));
            std::memcpy(&pc_message.data[i * pc_message.point_step + sizeof(float)], &y, sizeof(float));
        }
        debug_points_pub_->publish(pc_message);
    }

    void publish_debug_map() {
        if (!map_acquired) {
            return;
        }
        nav_msgs::msg::OccupancyGrid expanded_map_msg = *map_;
        expanded_map_msg.data = std::vector<int8_t>(expanded_map_.begin<uchar>(), expanded_map_.end<uchar>());
        debug_map_pub_->publish(expanded_map_msg);
    }

    void publish_debug_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr original_scan,
                                 const std::vector<int>& free_indices) {
        sensor_msgs::msg::LaserScan scan_msg = *original_scan;
        for (int index : free_indices) {
            scan_msg.ranges[index] = 0.0;
        }
        debug_obstacles_pub_->publish(scan_msg);
    }

    void publish_obstacle_points(const std::vector<cv::Point2d>& points, const rclcpp::Time& stamp) {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = stamp;
        pose_array.header.frame_id = map_->header.frame_id;
        pose_array.poses.resize(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            pose_array.poses[i].position.x = points[i].x;
            pose_array.poses[i].position.y = points[i].y;
        }
        obstacles_pub_->publish(pose_array);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>> ts_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_points_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr debug_obstacles_pub_;

    bool map_acquired;
    bool tf_aquired;
    std::string map_topic_;
    std::string scan_topic_;
    std::string odom_topic_;
    std::string scan_frame_;
    std::string base_frame_;
    double safety_margin_;
    bool debug_;
    std::string debug_points_topic_;
    std::string debug_map_topic_;
    std::string debug_obstacles_topic_;
    cv::Mat expanded_map_;
    double laser_x_;
    double laser_y_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstaclesDetectionLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
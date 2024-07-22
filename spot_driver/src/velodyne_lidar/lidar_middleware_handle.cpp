// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/velodyne_lidar/lidar_middleware_handle.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <spot_driver/types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 1;
constexpr auto kNodeName{"lidar_publisher"};

// ROS topic names for Spot's robot state publisher
constexpr auto kLidarStatesTopic{"velodyne_lidar"};

}  // namespace

namespace spot_ros2 {

LidarMiddlewareHandle::LidarMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node)
    : node_{node},
      lidar_publisher_{node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          kLidarStatesTopic, makePublisherQoS(kPublisherHistoryDepth))} {}

LidarMiddlewareHandle::LidarMiddlewareHandle(const rclcpp::NodeOptions& node_options)
    : LidarMiddlewareHandle(std::make_shared<rclcpp::Node>(kNodeName, node_options)) {}

void LidarMiddlewareHandle::publishLidar(const sensor_msgs::msg::PointCloud2& msg) {
  lidar_publisher_->publish(msg);
}

}  // namespace spot_ros2

// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/velodyne_lidar/lidar_publisher.hpp>
#include <spot_driver/types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace spot_ros2 {

/**
 * @brief Production implementation of a StatePublisher::MiddlewareHandle
 */

class LidarMiddlewareHandle : public LidarPublisher::MiddlewareHandle {
 public:
  /**
   * @brief Constructor for LidarMiddlewareHandle.
   * @param node A shared_pr to an instance of rclcpp::Node.
   */
  explicit LidarMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Constructor for LidarMiddlewareHandle.
   * @details This constructor creates a new rclcpp::Node using the provided NodeOptions.
   * @param node_options configuration options for a rclcpp::Node.
   */
  explicit LidarMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  ~LidarMiddlewareHandle() override = default;

  /**
   * @brief Publish robot state messages
   * @param robot_state_msgs Robot state messages to publish
   */
  void publishLidar(const sensor_msgs::msg::PointCloud2& msg) override;

 private:
  /** @brief Shared instance of an rclcpp node to create publishers */
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> lidar_publisher_;

};

}  // namespace spot_ros2

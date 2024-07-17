// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/executors.hpp>
#include <spot_driver/velodyne_lidar/lidar_publisher_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  spot_ros2::LidarPublisherNode node;

  // Spins the node with the default single-threaded executor.
  rclcpp::spin(node.get_node_base_interface());

  return 0;
}

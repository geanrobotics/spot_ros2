// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/robot_state.pb.h>
#include <spot_driver/api/default_lidar_client.hpp>
#include <string>
#include <vector>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

LidarClient::LidarClient(::bosdyn::client::RobotStateClient* client) : client_{client} {}

tl::expected<bosdyn::api::RobotState, std::string> LidarClient::getPointCloud() {
  std::vector<std::string> point_cloud_sources = {"velodyne-point-cloud"};

  //return get_robot_state_result..robot_state();
  return tl::make_unexpected("get lidar: ");
}


}  // namespace spot_ros2

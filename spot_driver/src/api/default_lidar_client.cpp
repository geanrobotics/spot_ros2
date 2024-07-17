// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/robot_state.pb.h>
#include <spot_driver/api/default_state_client.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

DefaultStateClient::DefaultStateClient(::bosdyn::client::RobotStateClient* client) : client_{client} {}

tl::expected<bosdyn::api::RobotState, std::string> DefaultStateClient::getPointCloud() {
  const auto get_point_cloud_result = client_->GetPointCloudAsync(["velodyne-point-cloud"]).get();
  if (!get_robot_state_result.status) {
    return tl::make_unexpected("Failed to get lidar: " + get_point_cloud_result.status.DebugString());
  }

  //return get_robot_state_result..robot_state();
   return tl::make_unexpected("get lidar: " + get_point_cloud_result.response);
}


}  // namespace spot_ros2

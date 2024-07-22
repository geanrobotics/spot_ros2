// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot_state/robot_state_client.h>
#include <bosdyn/client/point_cloud/point_cloud_client.h>
#include <spot_driver/api/time_sync_api.hpp>

#include <spot_driver/interfaces/velodyne_client_interface.hpp>
#include <string>
#include <tl_expected/expected.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace spot_ros2 {

class LidarClient final : public LidarClientInterface {
 public:
  /**
   * @brief constructor for DefaultStateClient.
   *
   * @param client A pointer to Spot's RobotStateClient. A DefaultStateClient SHOULD NOT delete this pointer since it
   * does not take ownership.
   */
  explicit LidarClient(::bosdyn::client::PointCloudClient* client, std::shared_ptr<TimeSyncApi> time_sync_api,
                     const std::string& robot_name);

  /**
   * @brief Retrieve Spot's most recent robot state data.
   * @return Returns an expected which contains a RobotState message if the request was completed successfully or an
   * error message if the request could not be completed.
   */
  [[nodiscard]] tl::expected<sensor_msgs::msg::PointCloud2, std::string> getPointCloud() override;


 private:
  /** @brief A pointer to a RobotStateClient provided to this class during construction. */
  ::bosdyn::client::PointCloudClient* client_;
  std::shared_ptr<TimeSyncApi> time_sync_api_;
  std::string robot_name_;
};

}  // namespace spot_ros2
// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <spot_driver/api/default_lidar_client.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/velodyne_lidar/lidar_publisher.hpp>
#include <spot_driver/types.hpp>
#include <utility>

namespace {
constexpr auto kRobotStateCallbackPeriod = std::chrono::duration<double>{1.0 / 50.0};  // 50 Hz
}

namespace spot_ros2 {

LidarPublisher::LidarPublisher(const std::shared_ptr<LidarClientInterface>& lidar_client_interface,
                               const std::shared_ptr<TimeSyncApi>& time_sync_api,
                               std::unique_ptr<MiddlewareHandle> middleware_handle,
                               std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                               std::unique_ptr<LoggerInterfaceBase> logger_interface,
                               std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface,
                               std::unique_ptr<TimerInterfaceBase> timer_interface)
    : is_using_vision_{false},
      lidar_client_interface_{lidar_client_interface},
      time_sync_interface_{time_sync_api},
      middleware_handle_{std::move(middleware_handle)},
      parameter_interface_{std::move(parameter_interface)},
      logger_interface_{std::move(logger_interface)},
      tf_broadcaster_interface_{std::move(tf_broadcaster_interface)},
      timer_interface_{std::move(timer_interface)} {
  const auto spot_name = parameter_interface_->getSpotName();
  frame_prefix_ = spot_name.empty() ? "" : spot_name + "/";

  const auto preferred_odom_frame = parameter_interface_->getPreferredOdomFrame();
  is_using_vision_ = preferred_odom_frame == "vision";
  full_odom_frame_id_ =
      preferred_odom_frame.find('/') == std::string::npos ? frame_prefix_ + preferred_odom_frame : preferred_odom_frame;

  // Create a timer to request and publish robot state at a fixed rate
  timer_interface_->setTimer(kRobotStateCallbackPeriod, [this] {
    timerCallback();
  });
}

void LidarPublisher::timerCallback() {
  // Get latest clock skew each time we request a robot state
  const auto clock_skew_result = time_sync_interface_->getClockSkew();
  if (!clock_skew_result) {
    logger_interface_->logError(std::string{"Failed to get latest clock skew: "}.append(clock_skew_result.error()));
    return;
  }

  const auto lidar_result =lidar_client_interface_->getPointCloud();
  if (!lidar_result.has_value()) {
    logger_interface_->logError(std::string{"Failed to get lidar: "}.append(lidar_result.error()));
    return;
  }

  const auto& clock_skew = clock_skew_result.value();
  const auto& robot_state = lidar_result.value();

  // will have to mould result to fit pcl

  //middleware_handle_->publishLidar(robot_state);

}

}  // namespace spot_ros2

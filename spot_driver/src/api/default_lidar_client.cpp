#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/api/point_cloud.pb.h>
#include <bosdyn/client/point_cloud/point_cloud_client.h>
#include <tl_expected/expected.hpp>
#include <string>
#include "spot_driver/api/default_lidar_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <array>
#include <cstring>

#include <google/protobuf/duration.pb.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <spot_driver/api/default_time_sync_api.hpp>
#include <spot_driver/conversions/time.hpp>
#include <spot_driver/types.hpp>
#include <std_msgs/msg/header.hpp>

namespace spot_ros2 {

LidarClient::LidarClient(::bosdyn::client::PointCloudClient* client,
                                       std::shared_ptr<TimeSyncApi> time_sync_api, const std::string& robot_name)
    : client_{client}, time_sync_api_{time_sync_api}, robot_name_{robot_name} {}

tl::expected<sensor_msgs::msg::PointCloud2, std::string> create_pointcloud2_message(
    const bosdyn::api::PointCloud& response, const std::string& robot_name,
    const google::protobuf::Duration& clock_skew) {

    sensor_msgs::msg::PointCloud2 msg;

    // Create header
    msg.header.frame_id =
      (robot_name.empty() ? "" : robot_name + "/") + response.source().frame_name_sensor();
    msg.header.stamp = spot_ros2::robotTimeToLocalTime(response.source().acquisition_time(), clock_skew);

    msg.height = 1;
    msg.width = response.num_points();
    msg.is_dense = false;
    msg.is_bigendian = false;

    // Define the fields of the point cloud
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.point_step = 12; // Each point consists of three 4-byte float32s
    msg.row_step = msg.point_step * response.num_points();

    // Convert points to byte array
    const std::string& data = response.data();
    msg.data.resize(data.size());
    std::memcpy(msg.data.data(), data.data(), data.size());

    return msg;
}

tl::expected<sensor_msgs::msg::PointCloud2, std::string> LidarClient::getPointCloud() {
    // Create the GetPointCloudRequest
    ::bosdyn::api::GetPointCloudRequest request;
    request.add_point_cloud_requests()->set_point_cloud_source_name("velodyne-point-cloud");

    // Call the asynchronous method and get the future
    std::shared_future<::bosdyn::client::GetPointCloudResultType> future = client_->GetPointCloudAsync(request);

    try {
        // Wait for the future to complete and get the result
        ::bosdyn::client::GetPointCloudResultType result = future.get();
        
        // Check if there are any point cloud responses
        if (result.response.point_cloud_responses().empty()) {
            return tl::make_unexpected("No point cloud responses received.");
        }

        // Process the first point cloud response
        const auto& response = result.response.point_cloud_responses(0);
        const bosdyn::api::PointCloud bdi_response = response.point_cloud();
        const auto clock_skew_result = time_sync_api_->getClockSkew();
        
        // Check if clock skew result is valid
        if (!clock_skew_result) {
            return tl::make_unexpected("Failed to get clock skew.");
        }

        // Create the PointCloud2 message
        const auto msg = create_pointcloud2_message(bdi_response, robot_name_, clock_skew_result.value());
        return msg;
        
    } catch (const std::exception& e) {
        return tl::make_unexpected(std::string("Error getting point cloud: ") + e.what());
    }
}

}  // namespace spot_ros2

#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/api/point_cloud.pb.h>
#include <bosdyn/client/point_cloud/point_cloud_client.h>
#include <tl_expected/expected.hpp>
#include <string>
#include "spot_driver/api/default_lidar_client.hpp"

namespace spot_ros2 {

LidarClient::LidarClient(::bosdyn::client::PointCloudClient* client) : client_{client} {}

tl::expected<bosdyn::api::GetPointCloudResponse, std::string> LidarClient::getPointCloud() {
    // Create the GetPointCloudRequest
    ::bosdyn::api::GetPointCloudRequest request;
    request.add_point_cloud_requests()->set_point_cloud_source_name("velodyne-point-cloud");

    // Call the asynchronous method and get the future
    std::shared_future<::bosdyn::client::GetPointCloudResultType> future = client_->GetPointCloudAsync(request);

    try {
        // Wait for the future to complete and get the result
        ::bosdyn::client::GetPointCloudResultType result = future.get();
        for (const auto& response : result.response.point_cloud_responses()) {
            ::bosdyn::api::PointCloud pcl = response.point_cloud();
            // call another function to move the .data()to numpy to format a message type
            // for publishing onto a topic
        }
    } catch (const std::exception& e) {
        return tl::make_unexpected(std::string("Error getting point cloud: ") + e.what());
    }
}

}  // namespace spot_ros2

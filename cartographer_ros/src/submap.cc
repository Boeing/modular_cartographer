/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/submap.h"

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"

namespace cartographer_ros
{

std::unique_ptr<::cartographer::io::SubmapTextures>
    FetchSubmapTextures(const ::cartographer::mapping::SubmapId& submap_id,
                        rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client)
{
    auto req = std::make_shared<cartographer_ros_msgs::srv::SubmapQuery::Request>();
    req->trajectory_id = submap_id.trajectory_id;
    req->submap_index = submap_id.submap_index;
    auto res = client->async_send_request(req);
    if (res.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        throw std::runtime_error("Failed to call: " + std::string(client->get_service_name()));
    }
    auto submap_result = res.get();
    if (submap_result->status.code != ::cartographer_ros_msgs::msg::StatusCode::OK)
    {
        return nullptr;
    }
    if (submap_result->textures.empty())
    {
        return nullptr;
    }
    auto response = absl::make_unique<::cartographer::io::SubmapTextures>();
    response->version = submap_result->submap_version;
    for (const auto& texture : submap_result->textures)
    {
        const std::string compressed_cells(texture.cells.begin(), texture.cells.end());
        response->textures.emplace_back(::cartographer::io::SubmapTexture{
            ::cartographer::io::UnpackTextureData(compressed_cells, texture.width, texture.height), texture.width,
            texture.height, texture.resolution, ToRigid3d(texture.slice_pose)});
    }
    return response;
}

}  // namespace cartographer_ros

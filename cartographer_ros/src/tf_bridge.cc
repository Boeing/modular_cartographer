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

#include "cartographer_ros/tf_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"

namespace cartographer_ros
{

TfBridge::TfBridge(const std::string& tracking_frame, const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame), lookup_transform_timeout_sec_(lookup_transform_timeout_sec), buffer_(buffer)
{
}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(const ::cartographer::common::Time time,
                                                                               const std::string& frame_id) const
{
    // ::ros::Duration timeout(lookup_transform_timeout_sec_);
    tf2::Duration timeout(tf2::durationFromSec(lookup_transform_timeout_sec_));
    std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
    try
    {
        // const ::ros::Time latest_tf_time =
        const ::builtin_interfaces::msg::Time latest_tf_time =
            // buffer_->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
            buffer_->lookupTransform(tracking_frame_, frame_id, tf2::TimePointZero,
             timeout).header.stamp;
        // const ::ros::Time requested_time = ToRos(time);
        // if (latest_tf_time >= requested_time)
        const ::builtin_interfaces::msg::Time requested_time = ToRos(time);
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted{std::chrono::nanoseconds{requested_time.sec*1000000000LL+requested_time.nanosec}};
        std::chrono::system_clock::time_point recovered = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted);
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted_tf_time{std::chrono::nanoseconds{latest_tf_time.sec*1000000000LL+latest_tf_time.nanosec}};
        std::chrono::system_clock::time_point recovered_tf_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted_tf_time);

        if (recovered_tf_time >= recovered)        
        {
            // We already have newer data, so we do not wait. Otherwise, we would wait
            // for the full 'timeout' even if we ask for data that is too old.
            // timeout = ::ros::Duration(0.);
            timeout = tf2::durationFromSec(0.0);

        }
        return absl::make_unique<::cartographer::transform::Rigid3d>(
            ToRigid3d(buffer_->lookupTransform(
                // tracking_frame_, frame_id, requested_time, timeout)));
                tracking_frame_, frame_id, recovered, timeout)));
    }
    catch (const tf2::TransformException& ex)
    {
        LOG(WARNING) << "Failed to lookup " << frame_id << " -> " << tracking_frame_ << " " << ex.what();
    }
    return nullptr;
}

}  // namespace cartographer_ros

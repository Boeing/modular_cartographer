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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/msg/system_state.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include "cartographer_ros_msgs/srv/load_state.hpp"
#include "cartographer_ros_msgs/srv/read_metrics.hpp"
#include "cartographer_ros_msgs/srv/start_localisation.hpp"
#include "cartographer_ros_msgs/srv/start_mapping.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include <rmw/qos_profiles.h>

namespace cartographer_ros
{

// Wires up ROS topics to SLAM.
class Cartographer : public rclcpp::Node
{
  public:
    Cartographer(const NodeOptions& node_options, const TrajectoryOptions& trajectory_options,
                 const bool collect_metrics);
    ~Cartographer();

    void Reset() EXCLUSIVE_LOCKS_REQUIRED(mutex_);
    void StartTimerCallbacks();
    void CancelTimerCallbacks();

    void FinishAllTrajectories() LOCKS_EXCLUDED(mutex_);
    bool FinishTrajectory(int trajectory_id);
    void RunFinalOptimization() EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                               const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
    void HandleLandmarkMessage(int trajectory_id, const std::string& sensor_id,
                               const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr msg);
    void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void HandleMultiEchoLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                                         const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg);
    void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    void SerializeState(const std::string& filename, const bool include_unfinished_submaps);
    void LoadState(const std::string& state_filename, bool load_frozen_state);

    rclcpp::Node::SharedPtr node_handle_;

    int AddOfflineTrajectory(
        const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
        const TrajectoryOptions& options);

  private:
    struct Subscriber
    {
        std::shared_ptr<rclcpp::SubscriptionBase> subscriber;

        // ::ros::Subscriber::getTopic() does not necessarily return the same
        // std::string
        // it was given in its constructor. Since we rely on the topic name as the
        // unique identifier of a subscriber, we remember it ourselves.
        std::string topic;
    };

    void HandleSubmapQuery(const std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Request> request,
                           std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Response> response);
    void HandleTrajectoryQuery(const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
                               std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response);
    void HandleLoadState(const std::shared_ptr<cartographer_ros_msgs::srv::LoadState::Request> request,
                         std::shared_ptr<cartographer_ros_msgs::srv::LoadState::Response> response);
    void HandleWriteState(const std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Request> request,
                          std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Response> response);
    void HandleGetTrajectoryStates(
        const std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Request> request,
        std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Response> response);
    void HandleReadMetrics(const std::shared_ptr<cartographer_ros_msgs::srv::ReadMetrics::Request> request,
                           std::shared_ptr<cartographer_ros_msgs::srv::ReadMetrics::Response> response);

    void HandleStartLocalisation(const std::shared_ptr<cartographer_ros_msgs::srv::StartLocalisation::Request> request,
                                 std::shared_ptr<cartographer_ros_msgs::srv::StartLocalisation::Response> response)
        LOCKS_EXCLUDED(mutex_);
    void HandleStopLocalisation(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res) LOCKS_EXCLUDED(mutex_);

    void HandlePauseLocalisation(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) LOCKS_EXCLUDED(mutex_);
    void HandleResumeLocalisation(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res) LOCKS_EXCLUDED(mutex_);

    void HandleStartMapping(const std::shared_ptr<cartographer_ros_msgs::srv::StartMapping::Request> request,
                            std::shared_ptr<cartographer_ros_msgs::srv::StartMapping::Response> response)
        LOCKS_EXCLUDED(mutex_);
    void HandleStopMapping(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res) LOCKS_EXCLUDED(mutex_);

    void HandleMapData(const std_msgs::msg::UInt8MultiArray msg) LOCKS_EXCLUDED(mutex_);

    int AddTrajectory(const TrajectoryOptions& options);

    void PublishSubmapList();
    void PublishLocalTrajectoryData() LOCKS_EXCLUDED(mutex_);
    void PublishTrajectoryNodeList();
    void PublishLandmarkPosesList();
    void PublishConstraintList();

    // Not sure if this is correct originally passing "const ::ros::WallTimerEvent&", but doesn't exist anymore.
    void PausedTimer();

    cartographer_ros_msgs::msg::StatusResponse FinishTrajectoryUnderLock(int trajectory_id)
        EXCLUSIVE_LOCKS_REQUIRED(mutex_);

    // Not sure if this can be done in ROS2.
    // void MaybeWarnAboutTopicMismatch();

    // Helper function for service handlers that need to check trajectory states.
    cartographer_ros_msgs::msg::StatusResponse TrajectoryStateToStatus(
        int trajectory_id, const std::set<cartographer::mapping::PoseGraphInterface::TrajectoryState>& valid_states);

    const NodeOptions node_options_;
    const TrajectoryOptions trajectory_options_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    absl::Mutex mutex_;
    std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
    std::shared_ptr<MapBuilderBridge> map_builder_bridge_ GUARDED_BY(mutex_);

    std::string map_data_ GUARDED_BY(mutex_);
    cartographer_ros_msgs::msg::SystemState system_state_ GUARDED_BY(mutex_);

    ::cartographer::transform::Rigid3d paused_tracking_in_global_ GUARDED_BY(mutex_);
    ::cartographer::transform::Rigid3d paused_global_to_odom_ GUARDED_BY(mutex_);

    ::rclcpp::TimerBase::SharedPtr paused_timer_;

    ::rclcpp::Node::SharedPtr nh_;
    ::rclcpp::Node::SharedPtr p_nh_;

    ::rclcpp::Publisher<::cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;
    ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_node_list_publisher_;
    ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr landmark_poses_list_publisher_;
    ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr constraint_list_publisher_;
    ::rclcpp::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr mapping_occupancy_grid_publisher_;

    rclcpp::Service<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr submap_query_service_;
    rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr trajectory_query_service_;
    rclcpp::Service<cartographer_ros_msgs::srv::LoadState>::SharedPtr load_state_service_;
    rclcpp::Service<cartographer_ros_msgs::srv::WriteState>::SharedPtr write_state_service_;
    rclcpp::Service<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr get_trajectory_states_service_;
    rclcpp::Service<cartographer_ros_msgs::srv::ReadMetrics>::SharedPtr read_metrics_service_;
    rclcpp::Service<cartographer_ros_msgs::srv::StartLocalisation>::SharedPtr start_localisation_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_localisation_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_localisation_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_localisation_service_;
    rclcpp::Service<cartographer_ros_msgs::srv::StartMapping>::SharedPtr handle_start_mapping_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_stop_mapping_server_;

    ::rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;
    ::rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scan_features_publisher_;
    ::rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submap_features_publisher_;
    ::rclcpp::Publisher<cartographer_ros_msgs::msg::SystemState>::SharedPtr system_state_publisher_;
    ::rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr map_data_subscriber_;

    struct TrajectorySensorSamplers
    {
        TrajectorySensorSamplers(const double rangefinder_sampling_ratio, const double odometry_sampling_ratio,
                                 const double fixed_frame_pose_sampling_ratio, const double landmark_sampling_ratio)
            : rangefinder_sampler(rangefinder_sampling_ratio), odometry_sampler(odometry_sampling_ratio),
              fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio), landmark_sampler(landmark_sampling_ratio)
        {
        }

        ::cartographer::common::FixedRatioSampler rangefinder_sampler;
        ::cartographer::common::FixedRatioSampler odometry_sampler;
        ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
        ::cartographer::common::FixedRatioSampler landmark_sampler;
    };

    // These are keyed with 'trajectory_id'.
    std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
    std::unordered_map<int, std::vector<Subscriber>> subscribers_;
    std::unordered_set<int> trajectories_scheduled_for_finish_;

    // We have to keep the timer handles of ::rclcpp::TimerBase around, otherwise they do not fire
    // std::vector<::rclcpp::TimerBase> wall_timers_;
    ::rclcpp::TimerBase::SharedPtr submap_list_timer_;
    std::chrono::time_point<std::chrono::steady_clock> submap_list_timer_last_update;
    ::rclcpp::TimerBase::SharedPtr trajectory_states_timer_;
    std::chrono::time_point<std::chrono::steady_clock> trajectory_states_timer__last_update;
    ::rclcpp::TimerBase::SharedPtr trajectory_node_list_timer_;
    std::chrono::time_point<std::chrono::steady_clock> trajectory_node_list_timer_last_update;
    ::rclcpp::TimerBase::SharedPtr landmark_pose_list_timer_;
    std::chrono::time_point<std::chrono::steady_clock> landmark_pose_list_timer_update;
    ::rclcpp::TimerBase::SharedPtr constrain_list_timer_;
    std::chrono::time_point<std::chrono::steady_clock> constrain_list_timer_last_update;

    // rclcpp::QoS custom_qos_profile_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include "cartographer_ros/node.h"

#include <chrono>
#include <png.h>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"

#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"
#include <cartographer/mapping/map_builder.h>

#include <boost/interprocess/streams/vectorstream.hpp>
#include <boost/iostreams/stream.hpp>

#include "cartographer_ros/proto_sstream.h"

namespace cartographer_ros
{

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace
{
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(void (Node::*handler)(int, const std::string&,
                                                             const typename MessageType::ConstPtr&),
                                       const int trajectory_id, const std::string& topic,
                                       ::ros::NodeHandle* const node_handle, Node* const node)
{
    return node_handle->subscribe<MessageType>(
        topic, kInfiniteSubscriberQueueSize,
        boost::function<void(const typename MessageType::ConstPtr&)>(
            [node, handler, trajectory_id, topic](const typename MessageType::ConstPtr& msg) {
                (node->*handler)(trajectory_id, topic, msg);
            }));
}

std::string TrajectoryStateToString(const TrajectoryState trajectory_state)
{
    switch (trajectory_state)
    {
        case TrajectoryState::ACTIVE:
            return "ACTIVE";
        case TrajectoryState::FINISHED:
            return "FINISHED";
        case TrajectoryState::FROZEN:
            return "FROZEN";
        case TrajectoryState::DELETED:
            return "DELETED";
    }
    return "";
}

}  // namespace

Node::Node(const NodeOptions& node_options, const TrajectoryOptions& trajectory_options,
           const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const bool collect_metrics)

    : node_options_(node_options), trajectory_options_(trajectory_options), tf_buffer_(tf_buffer), p_nh_("~")
{
    absl::MutexLock lock(&mutex_);

    map_builder_bridge_ = std::make_unique<MapBuilderBridge>(node_options_, tf_buffer_);

    if (collect_metrics)
    {
        metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
        carto::metrics::RegisterAllMetrics(metrics_registry_.get());
    }

    submap_list_publisher_ =
        p_nh_.advertise<::cartographer_ros_msgs::SubmapList>(kSubmapListTopic, kLatestOnlyPublisherQueueSize);
    trajectory_node_list_publisher_ =
        p_nh_.advertise<::visualization_msgs::MarkerArray>(kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
    landmark_poses_list_publisher_ =
        p_nh_.advertise<::visualization_msgs::MarkerArray>(kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
    constraint_list_publisher_ =
        p_nh_.advertise<::visualization_msgs::MarkerArray>(kConstraintListTopic, kLatestOnlyPublisherQueueSize);
    occupancy_grid_publisher_ =
        p_nh_.advertise<::nav_msgs::OccupancyGrid>(kOccupancyGridTopic, kLatestOnlyPublisherQueueSize);

    service_servers_.push_back(p_nh_.advertiseService(kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
    service_servers_.push_back(p_nh_.advertiseService(kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));
    service_servers_.push_back(p_nh_.advertiseService(kLoadStateServiceName, &Node::HandleLoadState, this));
    service_servers_.push_back(p_nh_.advertiseService(kWriteStateServiceName, &Node::HandleWriteState, this));
    service_servers_.push_back(
        p_nh_.advertiseService(kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
    service_servers_.push_back(p_nh_.advertiseService(kReadMetricsServiceName, &Node::HandleReadMetrics, this));

    service_servers_.push_back(p_nh_.advertiseService("start_localisation", &Node::HandleStartLocalisation, this));
    service_servers_.push_back(p_nh_.advertiseService("stop_localisation", &Node::HandleStopLocalisation, this));

    service_servers_.push_back(p_nh_.advertiseService("start_mapping", &Node::HandleStartMapping, this));
    service_servers_.push_back(p_nh_.advertiseService("stop_mapping", &Node::HandleStopMapping, this));

    map_data_subscriber_ = p_nh_.subscribe<std_msgs::UInt8MultiArray>(kMapDataTopic, kInfiniteSubscriberQueueSize,
                                                                      &Node::HandleMapData, this);

    scan_matched_point_cloud_publisher_ =
        p_nh_.advertise<sensor_msgs::PointCloud2>(kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
    scan_features_publisher_ =
        p_nh_.advertise<visualization_msgs::MarkerArray>("scan_features", kLatestOnlyPublisherQueueSize);
    submap_features_publisher_ =
        p_nh_.advertise<visualization_msgs::MarkerArray>("submap_features", kLatestOnlyPublisherQueueSize);
}

Node::~Node()
{
    FinishAllTrajectories();
}

::ros::NodeHandle* Node::node_handle()
{
    return &nh_;
}

void Node::Reset()
{
    LOG(INFO) << "Resetting state";

    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        if (entry.second == TrajectoryState::ACTIVE)
        {
            const int trajectory_id = entry.first;
            FinishTrajectoryUnderLock(trajectory_id);
        }
    }

    wall_timers_.clear();
    sensor_samplers_.clear();
    subscribers_.clear();
    subscribed_topics_.clear();
    trajectories_scheduled_for_finish_.clear();

    map_builder_bridge_->RunFinalOptimization();

    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        LOG(INFO) << "Trajectory: " << entry.first << " state: " << static_cast<int>(entry.second);
    }

    LOG(INFO) << " num_trajectory_builders: " << map_builder_bridge_->map_builder().num_trajectory_builders();

    LOG(INFO) << "Reset complete";

    map_builder_bridge_ = std::make_unique<MapBuilderBridge>(node_options_, tf_buffer_);

    if (!map_data_.empty())
    {
        try
        {
            boost::iostreams::stream<boost::iostreams::array_source> ss(map_data_.c_str(), map_data_.size());
            map_builder_bridge_->LoadState(ss, true);
        }
        catch (const std::exception& e)
        {
            LOG(ERROR) << "Failed to load map data: " << e.what();
        }
    }
}

void Node::StartTimerCallbacks()
{
    wall_timers_.clear();
    wall_timers_.push_back(nh_.createWallTimer(::ros::WallDuration(node_options_.submap_publish_period_sec),
                                               &Node::PublishSubmapList, this));
    if (node_options_.pose_publish_period_sec > 0)
    {
        publish_local_trajectory_data_timer_ = nh_.createTimer(::ros::Duration(node_options_.pose_publish_period_sec),
                                                               &Node::PublishLocalTrajectoryData, this);
    }
    wall_timers_.push_back(nh_.createWallTimer(::ros::WallDuration(node_options_.trajectory_publish_period_sec),
                                               &Node::PublishTrajectoryNodeList, this));
    wall_timers_.push_back(nh_.createWallTimer(::ros::WallDuration(node_options_.trajectory_publish_period_sec),
                                               &Node::PublishLandmarkPosesList, this));
    wall_timers_.push_back(
        nh_.createWallTimer(::ros::WallDuration(kConstraintPublishPeriodSec), &Node::PublishConstraintList, this));
}

bool Node::HandleSubmapQuery(::cartographer_ros_msgs::SubmapQuery::Request& request,
                             ::cartographer_ros_msgs::SubmapQuery::Response& response)
{
    absl::MutexLock lock(&mutex_);
    map_builder_bridge_->HandleSubmapQuery(request, response);
    return true;
}

bool Node::HandleTrajectoryQuery(::cartographer_ros_msgs::TrajectoryQuery::Request& request,
                                 ::cartographer_ros_msgs::TrajectoryQuery::Response& response)
{
    absl::MutexLock lock(&mutex_);
    response.status =
        TrajectoryStateToStatus(request.trajectory_id, {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
                                                        TrajectoryState::FROZEN} /* valid states */);
    if (response.status.code != cartographer_ros_msgs::StatusCode::OK)
    {
        LOG(ERROR) << "Can't query trajectory from pose graph: " << response.status.message;
        return true;
    }
    map_builder_bridge_->HandleTrajectoryQuery(request, response);
    return true;
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent&)
{
    absl::MutexLock lock(&mutex_);

    const auto submaps = map_builder_bridge_->GetSubmapList();
    submap_list_publisher_.publish(submaps);

    // This is very CPU intensive
    //    const nav_msgs::OccupancyGrid og_map = map_builder_bridge_->GetOccupancyGridMsg(0.1);
    //    occupancy_grid_publisher_.publish(og_map);

    if (submap_features_publisher_.getNumSubscribers() > 0)
    {
        const auto submap_data = map_builder_bridge_->map_builder().pose_graph()->GetAllSubmapData();

        auto MakeCylinder = [](ros::Time time, const double radius, const int index, const std::string& frame_id,
                               const Eigen::Vector2f& position) {
            visualization_msgs::Marker marker;
            marker.ns = "Features";
            marker.id = index;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.header.stamp = time;
            marker.header.frame_id = frame_id;
            marker.scale.x = radius * 2.0;
            marker.scale.y = radius * 2.0;
            marker.scale.z = 0.6;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = marker.scale.z / 2.0;
            marker.pose.orientation.w = 1.0;
            return marker;
        };

        visualization_msgs::MarkerArray feature_markers;
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        feature_markers.markers.push_back(delete_all);
        const ros::Time feature_time = ros::Time::now();
        for (const auto item : submap_data)
        {
            const auto& sm = dynamic_cast<const cartographer::mapping::Submap2D&>(*item.data.submap);
            for (const auto& f : sm.CircleFeatures())
            {
                feature_markers.markers.push_back(MakeCylinder(feature_time, f.fdescriptor.radius,
                                                               static_cast<int>(feature_markers.markers.size()),
                                                               node_options_.map_frame, f.keypoint.position.head<2>()));
            }
        }

        submap_features_publisher_.publish(feature_markers);
    }
}

void Node::AddSensorSamplers(const int trajectory_id, const TrajectoryOptions& options)
{
    CHECK(sensor_samplers_.count(trajectory_id) == 0);
    sensor_samplers_.emplace(std::piecewise_construct, std::forward_as_tuple(trajectory_id),
                             std::forward_as_tuple(options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
                                                   options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
                                                   options.landmarks_sampling_ratio));
}

void Node::PublishLocalTrajectoryData(const ::ros::TimerEvent&)
{
    absl::MutexLock lock(&mutex_);

    for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData())
    {
        const auto& trajectory_data = entry.second;

        if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0)
        {
            // TODO(gaschler): Consider using other message without time information
            carto::sensor::TimedPointCloud point_cloud;
            point_cloud.reserve(trajectory_data.local_slam_data->trajectory_node_data->range_data.returns.size());
            std::transform(trajectory_data.local_slam_data->trajectory_node_data->range_data.returns.begin(),
                           trajectory_data.local_slam_data->trajectory_node_data->range_data.returns.end(),
                           std::back_inserter(point_cloud), [](const cartographer::sensor::RangefinderPoint& point) {
                               return cartographer::sensor::ToTimedRangefinderPoint(point, 0.f /* time */);
                           });
            scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
                carto::common::ToUniversal(trajectory_data.local_slam_data->time),
                trajectory_data.trajectory_options.tracking_frame,
                carto::sensor::TransformTimedPointCloud(point_cloud, trajectory_data.local_to_map.cast<float>())));
        }

        if (scan_features_publisher_.getNumSubscribers() > 0)
        {
            auto MakeCylinder = [](ros::Time time, const double radius, const int index, const std::string& frame_id,
                                   const Eigen::Vector2f& position) {
                visualization_msgs::Marker marker;
                marker.ns = "Features";
                marker.id = index;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.header.stamp = time;
                marker.header.frame_id = frame_id;
                marker.scale.x = radius * 2.0;
                marker.scale.y = radius * 2.0;
                marker.scale.z = 0.6;
                marker.color.r = 1.0;
                marker.color.a = 1.0;
                marker.pose.position.x = position.x();
                marker.pose.position.y = position.y();
                marker.pose.position.z = marker.scale.z / 2.0;
                marker.pose.orientation.w = 1.0;
                return marker;
            };

            visualization_msgs::MarkerArray feature_markers;
            visualization_msgs::Marker delete_all;
            delete_all.action = visualization_msgs::Marker::DELETEALL;
            feature_markers.markers.push_back(delete_all);
            const ros::Time feature_time = ToRos(trajectory_data.local_slam_data->time);
            for (const auto& feature : trajectory_data.local_slam_data->trajectory_node_data->circle_features)
            {
                feature_markers.markers.push_back(MakeCylinder(
                    feature_time, feature.fdescriptor.radius, static_cast<int>(feature_markers.markers.size()),
                    trajectory_data.trajectory_options.tracking_frame, feature.keypoint.position.head<2>()));
            }

            scan_features_publisher_.publish(feature_markers);
        }

        geometry_msgs::TransformStamped stamped_transform;

        stamped_transform.header.stamp = ros::Time::now();  // ToRos(trajectory_data.local_slam_data->time);

        // TODO if all well and good then publish time
        // if localisation is no good then don't publish

        const Rigid3d tracking_to_local_3d = trajectory_data.local_slam_data->local_pose;
        const Rigid3d tracking_to_local = [&] {
            if (trajectory_data.trajectory_options.publish_frame_projected_to_2d)
            {
                return carto::transform::Embed3D(carto::transform::Project2D(tracking_to_local_3d));
            }
            return tracking_to_local_3d;
        }();

        const Rigid3d tracking_to_map = trajectory_data.local_to_map * tracking_to_local;

        if (trajectory_data.published_to_tracking != nullptr)
        {
            if (trajectory_data.trajectory_options.provide_odom_frame)
            {
                std::vector<geometry_msgs::TransformStamped> stamped_transforms;

                stamped_transform.header.frame_id = node_options_.map_frame;
                stamped_transform.child_frame_id = trajectory_data.trajectory_options.odom_frame;
                stamped_transform.transform = ToGeometryMsgTransform(trajectory_data.local_to_map);
                stamped_transforms.push_back(stamped_transform);

                stamped_transform.header.frame_id = trajectory_data.trajectory_options.odom_frame;
                stamped_transform.child_frame_id = trajectory_data.trajectory_options.published_frame;
                stamped_transform.transform =
                    ToGeometryMsgTransform(tracking_to_local * (*trajectory_data.published_to_tracking));
                stamped_transforms.push_back(stamped_transform);

                tf_broadcaster_.sendTransform(stamped_transforms);
            }
            else
            {
                stamped_transform.header.frame_id = node_options_.map_frame;                            // map
                stamped_transform.child_frame_id = trajectory_data.trajectory_options.published_frame;  // odom
                stamped_transform.transform =
                    ToGeometryMsgTransform(tracking_to_map * (*trajectory_data.published_to_tracking));
                tf_broadcaster_.sendTransform(stamped_transform);
            }
        }
    }
}

void Node::PublishTrajectoryNodeList(const ::ros::WallTimerEvent&)
{
    if (trajectory_node_list_publisher_.getNumSubscribers() > 0)
    {
        absl::MutexLock lock(&mutex_);
        trajectory_node_list_publisher_.publish(map_builder_bridge_->GetTrajectoryNodeList());
    }
}

void Node::PublishLandmarkPosesList(const ::ros::WallTimerEvent&)
{
    if (landmark_poses_list_publisher_.getNumSubscribers() > 0)
    {
        absl::MutexLock lock(&mutex_);
        landmark_poses_list_publisher_.publish(map_builder_bridge_->GetLandmarkPosesList());
    }
}

void Node::PublishConstraintList(const ::ros::WallTimerEvent&)
{
    if (constraint_list_publisher_.getNumSubscribers() > 0)
    {
        absl::MutexLock lock(&mutex_);
        constraint_list_publisher_.publish(map_builder_bridge_->GetConstraintList());
    }
}

int Node::AddTrajectory(const TrajectoryOptions& options)
{
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> expected_sensor_ids =
        ComputeExpectedSensorIds(options);
    const int trajectory_id = map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
    AddSensorSamplers(trajectory_id, options);
    LaunchSubscribers(options, trajectory_id);
    wall_timers_.push_back(nh_.createWallTimer(::ros::WallDuration(kTopicMismatchCheckDelaySec),
                                               &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
    for (const auto& sensor_id : expected_sensor_ids)
    {
        subscribed_topics_.insert(sensor_id.id);
    }
    return trajectory_id;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
    const TrajectoryOptions& options)
{
    absl::MutexLock lock(&mutex_);
    const int trajectory_id = map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
    AddSensorSamplers(trajectory_id, options);
    return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options, const int trajectory_id)
{
    for (const std::string& topic : ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans))
    {
        subscribers_[trajectory_id].push_back({SubscribeWithHandler<sensor_msgs::LaserScan>(
                                                   &Node::HandleLaserScanMessage, trajectory_id, topic, &nh_, this),
                                               topic});
    }
    for (const std::string& topic :
         ComputeRepeatedTopicNames(kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans))
    {
        subscribers_[trajectory_id].push_back(
            {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(&Node::HandleMultiEchoLaserScanMessage,
                                                                   trajectory_id, topic, &nh_, this),
             topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds))
    {
        subscribers_[trajectory_id].push_back({SubscribeWithHandler<sensor_msgs::PointCloud2>(
                                                   &Node::HandlePointCloud2Message, trajectory_id, topic, &nh_, this),
                                               topic});
    }

    // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is required
    if (options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data())
    {
        subscribers_[trajectory_id].push_back(
            {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage, trajectory_id, kImuTopic, &nh_, this),
             kImuTopic});
    }

    if (options.use_odometry)
    {
        subscribers_[trajectory_id].push_back(
            {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage, trajectory_id, kOdometryTopic, &nh_,
                                                      this),
             kOdometryTopic});
    }
    if (options.use_nav_sat)
    {
        subscribers_[trajectory_id].push_back(
            {SubscribeWithHandler<sensor_msgs::NavSatFix>(&Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
                                                          &nh_, this),
             kNavSatFixTopic});
    }
    if (options.use_landmarks)
    {
        subscribers_[trajectory_id].push_back(
            {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(&Node::HandleLandmarkMessage, trajectory_id,
                                                                       kLandmarkTopic, &nh_, this),
             kLandmarkTopic});
    }
}

bool Node::ValidateTopicNames(const TrajectoryOptions& options)
{
    for (const auto& sensor_id : ComputeExpectedSensorIds(options))
    {
        const std::string& topic = sensor_id.id;
        if (subscribed_topics_.count(topic) > 0)
        {
            LOG(ERROR) << "Topic name [" << topic << "] is already used.";
            return false;
        }
    }
    return true;
}

cartographer_ros_msgs::StatusResponse Node::TrajectoryStateToStatus(const int trajectory_id,
                                                                    const std::set<TrajectoryState>& valid_states)
{
    const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
    cartographer_ros_msgs::StatusResponse status_response;

    const auto it = trajectory_states.find(trajectory_id);
    if (it == trajectory_states.end())
    {
        status_response.message = absl::StrCat("Trajectory ", trajectory_id, " doesn't exist.");
        status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
        return status_response;
    }

    status_response.message =
        absl::StrCat("Trajectory ", trajectory_id, " is in '", TrajectoryStateToString(it->second), "' state.");
    status_response.code = valid_states.count(it->second) ? cartographer_ros_msgs::StatusCode::OK
                                                          : cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    return status_response;
}

cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(const int trajectory_id)
{
    cartographer_ros_msgs::StatusResponse status_response;
    if (trajectories_scheduled_for_finish_.count(trajectory_id))
    {
        status_response.message = absl::StrCat("Trajectory ", trajectory_id, " already pending to finish.");
        status_response.code = cartographer_ros_msgs::StatusCode::OK;
        LOG(INFO) << status_response.message;
        return status_response;
    }

    // First, check if we can actually finish the trajectory.
    status_response = TrajectoryStateToStatus(trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
    if (status_response.code != cartographer_ros_msgs::StatusCode::OK)
    {
        LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
        return status_response;
    }

    // Shutdown the subscribers of this trajectory.
    // A valid case with no subscribers is e.g. if we just visualize states.
    if (subscribers_.count(trajectory_id))
    {
        for (auto& entry : subscribers_[trajectory_id])
        {
            entry.subscriber.shutdown();
            subscribed_topics_.erase(entry.topic);
            LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
        }
        CHECK_EQ(subscribers_.erase(trajectory_id), 1);
    }
    map_builder_bridge_->FinishTrajectory(trajectory_id);
    trajectories_scheduled_for_finish_.emplace(trajectory_id);
    status_response.message = absl::StrCat("Finished trajectory ", trajectory_id, ".");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    return status_response;
}

bool Node::HandleLoadState(cartographer_ros_msgs::LoadState::Request& request,
                           cartographer_ros_msgs::LoadState::Response& response)
{
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Loading state";

    std::string s(reinterpret_cast<char*>(request.pbstream_data.data()), request.pbstream_data.size());
    std::istringstream ss(s);

    map_builder_bridge_->LoadState(ss, request.load_frozen_state);

    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    return true;
}

bool Node::HandleGetTrajectoryStates(::cartographer_ros_msgs::GetTrajectoryStates::Request&,
                                     ::cartographer_ros_msgs::GetTrajectoryStates::Response& response)
{
    using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
    absl::MutexLock lock(&mutex_);
    response.status.code = ::cartographer_ros_msgs::StatusCode::OK;
    response.trajectory_states.header.stamp = ros::Time::now();
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        response.trajectory_states.trajectory_id.push_back(entry.first);
        switch (entry.second)
        {
            case TrajectoryState::ACTIVE:
                response.trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::TrajectoryStates::ACTIVE);
                break;
            case TrajectoryState::FINISHED:
                response.trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::TrajectoryStates::FINISHED);
                break;
            case TrajectoryState::FROZEN:
                response.trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::TrajectoryStates::FROZEN);
                break;
            case TrajectoryState::DELETED:
                response.trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::TrajectoryStates::DELETED);
                break;
        }
    }
    return true;
}

bool Node::HandleWriteState(::cartographer_ros_msgs::WriteState::Request& request,
                            ::cartographer_ros_msgs::WriteState::Response& response)
{
    absl::MutexLock lock(&mutex_);

    nav_msgs::OccupancyGrid og_map = map_builder_bridge_->GetOccupancyGridMsg(request.resolution);

    LOG(INFO) << "og_map: " << og_map.data.size();

    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        if (entry.second == TrajectoryState::ACTIVE)
        {
            const int trajectory_id = entry.first;
            CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code, cartographer_ros_msgs::StatusCode::OK);
        }
    }

    map_builder_bridge_->RunFinalOptimization();

    {
        for (const auto& traj : map_builder_bridge_->map_builder().pose_graph()->GetTrajectoryStates())
        {
            LOG(INFO) << "LOADED: trajectory_id: " << traj.first << " state: " << static_cast<int>(traj.second);
        }

        for (const auto& sm : map_builder_bridge_->map_builder().pose_graph()->GetAllSubmapPoses())
        {
            LOG(INFO) << "LOADED: submap: " << sm.id << " pose: " << sm.data.pose;
        }

        for (const auto& c : map_builder_bridge_->map_builder().pose_graph()->constraints())
        {
            LOG(INFO) << "LOADED: constraint: submap_id: " << c.submap_id << " node_id: " << c.node_id;
        }
    }

    response.map_info = og_map.info;

    std::stringstream ss;
    if (map_builder_bridge_->SerializeState(ss, false))
    {
        ss.seekp(0);
        ss >> std::noskipws;
        std::copy(std::istream_iterator<uint8_t>(ss), std::istream_iterator<uint8_t>(),
                  std::back_inserter(response.pbstream_data));

        {
            response.occupancy_grid.format = "png";

            png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
            png_infop info = png_create_info_struct(png);

            if (setjmp(png_jmpbuf(png)))
                abort();

            auto user_write_fn = [](png_structp png, png_bytep data, png_size_t length) {
                auto v = (sensor_msgs::CompressedImage::_data_type*)png_get_io_ptr(png);
                for (unsigned int i = 0; i < length; ++i)
                {
                    v->push_back(*data++);
                }
            };

            png_set_write_fn(png, (void*)&response.occupancy_grid.data, user_write_fn, nullptr);

            png_set_IHDR(png, info, og_map.info.width, og_map.info.height, 8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                         PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

            png_color_8 sig_bit;
            sig_bit.gray = 8;
            png_set_sBIT(png, info, &sig_bit);

            png_write_info(png, info);

            const int bytes_per_pixel = 1;

            if (og_map.info.height > PNG_SIZE_MAX / (og_map.info.width * bytes_per_pixel))
                png_error(png, "Image data buffer would be too large");

            png_bytep row_pointers[og_map.info.height];

            if (og_map.info.height > PNG_UINT_32_MAX / (sizeof(png_bytep)))
                png_error(png, "Image is too tall to process in memory");

            for (png_uint_32 k = 0; k < og_map.info.height; k++)
                row_pointers[k] =
                    reinterpret_cast<unsigned char*>(&og_map.data[0]) + k * og_map.info.width * bytes_per_pixel;

            png_write_image(png, row_pointers);

            png_write_end(png, info);

            png_destroy_write_struct(&png, &info);
        }

        response.status.code = cartographer_ros_msgs::StatusCode::OK;
        response.status.message = absl::StrCat("Success");
    }
    else
    {
        response.status.code = cartographer_ros_msgs::StatusCode::CANCELLED;
        response.status.message = absl::StrCat("Failed");
    }


    for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData())
    {
        const auto& trajectory_data = entry.second;
        const Rigid3d tracking_to_local_3d = trajectory_data.local_slam_data->local_pose;
        const Rigid3d tracking_to_local = [&] {
            if (trajectory_data.trajectory_options.publish_frame_projected_to_2d)
            {
                return carto::transform::Embed3D(carto::transform::Project2D(tracking_to_local_3d));
            }
            return tracking_to_local_3d;
        }();
        const Rigid3d tracking_to_map = trajectory_data.local_to_map * tracking_to_local;
        const Rigid3d map_to_odom = tracking_to_map * (*trajectory_data.published_to_tracking);

        response.trajectory_id.push_back(entry.first);
        response.pose.push_back(ToGeometryMsgPose(map_to_odom));
    }

    Reset();

    return true;
}

bool Node::HandleReadMetrics(::cartographer_ros_msgs::ReadMetrics::Request&,
                             ::cartographer_ros_msgs::ReadMetrics::Response& response)
{
    absl::MutexLock lock(&mutex_);
    response.timestamp = ros::Time::now();
    if (!metrics_registry_)
    {
        response.status.code = cartographer_ros_msgs::StatusCode::UNAVAILABLE;
        response.status.message = "Collection of runtime metrics is not activated.";
        return true;
    }
    metrics_registry_->ReadMetrics(&response);
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "Successfully read metrics.";
    return true;
}

bool Node::HandleStartLocalisation(cartographer_ros_msgs::StartLocalisation::Request& request,
                                   cartographer_ros_msgs::StartLocalisation::Response& response)
{
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to start localisation";

    Reset();

    TrajectoryOptions trajectory_options = trajectory_options_;

    auto trimmer = trajectory_options.trajectory_builder_options.mutable_pure_localization_trimmer();
    trimmer->set_max_submaps_to_keep(4);

    if (request.use_initial_pose)
    {
        const auto pose = ToRigid3d(request.initial_pose);
        if (!pose.IsValid())
        {
            response.status.message = "Invalid pose argument. Orientation quaternion must be normalized.";
            LOG(ERROR) << response.status.message;
            response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
            return true;
        }

        // Check if the requested trajectory for the relative initial pose exists.
        response.status = TrajectoryStateToStatus(
            request.relative_to_trajectory_id,
            {TrajectoryState::ACTIVE, TrajectoryState::FROZEN, TrajectoryState::FINISHED} /* valid states */);
        if (response.status.code != cartographer_ros_msgs::StatusCode::OK)
        {
            LOG(ERROR) << "Can't start a trajectory with initial pose: " << response.status.message;
            return true;
        }

        ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
        initial_trajectory_pose.set_to_trajectory_id(request.relative_to_trajectory_id);
        *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(pose);
        initial_trajectory_pose.set_timestamp(
            cartographer::common::ToUniversal(::cartographer_ros::FromRos(ros::Time(0))));
        *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;
    }

    if (!ValidateTopicNames(trajectory_options))
    {
        response.status.message = "Topics are already used by another trajectory.";
        LOG(ERROR) << response.status.message;
        response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    }
    else
    {
        response.status.message = "Success.";
        response.trajectory_id = AddTrajectory(trajectory_options);
        response.status.code = cartographer_ros_msgs::StatusCode::OK;
    }

    StartTimerCallbacks();

    return true;
}

bool Node::HandleStopLocalisation(std_srvs::TriggerRequest&, std_srvs::TriggerResponse&)
{
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to stop localisation";

    Reset();

    return true;
}

bool Node::HandleStartMapping(cartographer_ros_msgs::StartMapping::Request&,
                              cartographer_ros_msgs::StartMapping::Response& response)
{
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to start mapping";

    map_data_.clear();

    Reset();

    if (!ValidateTopicNames(trajectory_options_))
    {
        response.status.message = "Topics are already used by another trajectory.";
        LOG(ERROR) << response.status.message;
        response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    }
    else
    {
        response.status.message = "Success.";
        response.trajectory_id = AddTrajectory(trajectory_options_);
        response.status.code = cartographer_ros_msgs::StatusCode::OK;
    }

    StartTimerCallbacks();

    return true;
}

bool Node::HandleStopMapping(std_srvs::TriggerRequest&, std_srvs::TriggerResponse&)
{
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to stop mapping";

    Reset();

    return true;
}

void Node::HandleMapData(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);

    map_data_ = std::string(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

    Reset();

    LOG(INFO) << "Loading incoming map data (" << msg->data.size() << ")...";

    // auto start localising
    TrajectoryOptions trajectory_options = trajectory_options_;
    auto trimmer = trajectory_options.trajectory_builder_options.mutable_pure_localization_trimmer();
    trimmer->set_max_submaps_to_keep(4);
    AddTrajectory(trajectory_options);
    StartTimerCallbacks();
}

void Node::FinishAllTrajectories()
{
    absl::MutexLock lock(&mutex_);
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        if (entry.second == TrajectoryState::ACTIVE)
        {
            const int trajectory_id = entry.first;
            CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code, cartographer_ros_msgs::StatusCode::OK);
        }
    }
}

bool Node::FinishTrajectory(const int trajectory_id)
{
    absl::MutexLock lock(&mutex_);
    return FinishTrajectoryUnderLock(trajectory_id).code == cartographer_ros_msgs::StatusCode::OK;
}

void Node::RunFinalOptimization()
{
    {
        for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
        {
            const int trajectory_id = entry.first;
            if (entry.second == TrajectoryState::ACTIVE)
            {
                LOG(WARNING) << "Can't run final optimization if there are one or more active "
                                "trajectories. Trying to finish trajectory with ID "
                             << std::to_string(trajectory_id) << " now.";
                CHECK(FinishTrajectory(trajectory_id))
                    << "Failed to finish trajectory with ID " << std::to_string(trajectory_id) << ".";
            }
        }
    }
    // Assuming we are not adding new data anymore, the final optimization
    // can be performed without holding the mutex.
    map_builder_bridge_->RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id, const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse())
    {
        return;
    }
    auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
    sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id, const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(const int trajectory_id, const std::string& sensor_id,
                                 const cartographer_ros_msgs::LandmarkList::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleLandmarkMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id, const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);

    if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse())
    {
        return;
    }
    auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
    sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id, const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(const int trajectory_id, const std::string& sensor_id,
                                           const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(const int trajectory_id, const std::string& sensor_id,
                                    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename, const bool include_unfinished_submaps)
{
    absl::MutexLock lock(&mutex_);
    std::ofstream file(filename);
    CHECK(map_builder_bridge_->SerializeState(file, include_unfinished_submaps)) << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename, const bool load_frozen_state)
{
    absl::MutexLock lock(&mutex_);
    const std::string suffix = ".pbstream";
    CHECK_EQ(state_filename.substr(std::max<int>(state_filename.size() - suffix.size(), 0)), suffix)
        << "The file containing the state to be loaded must be a .pbstream file.";
    std::ifstream file(state_filename);
    map_builder_bridge_->LoadState(file, load_frozen_state);
}

void Node::MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&)
{
    ::ros::master::V_TopicInfo ros_topics;
    ::ros::master::getTopics(ros_topics);
    std::set<std::string> published_topics;
    std::stringstream published_topics_string;
    for (const auto& it : ros_topics)
    {
        std::string resolved_topic = nh_.resolveName(it.name, false);
        published_topics.insert(resolved_topic);
        published_topics_string << resolved_topic << ",";
    }
    bool print_topics = false;
    for (const auto& entry : subscribers_)
    {
        int trajectory_id = entry.first;
        for (const auto& subscriber : entry.second)
        {
            std::string resolved_topic = nh_.resolveName(subscriber.topic);
            if (published_topics.count(resolved_topic) == 0)
            {
                LOG(WARNING) << "Expected topic \"" << subscriber.topic << "\" (trajectory " << trajectory_id << ")"
                             << " (resolved topic \"" << resolved_topic << "\")"
                             << " but no publisher is currently active.";
                print_topics = true;
            }
        }
    }
    if (print_topics)
    {
        LOG(WARNING) << "Currently available topics are: " << published_topics_string.str();
    }
}

}  // namespace cartographer_ros

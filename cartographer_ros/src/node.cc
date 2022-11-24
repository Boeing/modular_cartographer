#include "cartographer_ros/node.h"

// #include <boost/interprocess/streams/vectorstream.hpp>
// #include <boost/iostreams/stream.hpp>
#include <cartographer/mapping/map_builder.h>
#include <nav_msgs/msg/odometry.hpp>
#include <png.h>

#include <chrono>
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
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/proto_sstream.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "glog/logging.h"
// #include "ros/serialization.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace cartographer_ros
{

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace
{
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
// template <typename MessageType>
// rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(void (Cartographer::*handler)(int, const std::string&,
//                     const typename MessageType::ConstSharedPtr),
//                     const int trajectory_id, const std::string& topic,
//                     ::rclcpp::Node::SharedPtr node_handle, Cartographer* const node, rclcpp::QoS custom_qos_profile_)
// {
//     return node_handle->create_subscription<MessageType>(
//       topic,
//       [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
//                 (node->*handler)(trajectory_id, topic, msg);
//             }, custom_qos_profile_);
// }

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

visualization_msgs::msg::Marker MakeCylinder(rclcpp::Time time, const double radius, const int index,
                                             const std::string& frame_id, const Eigen::Vector2f& position,
                                             const float r, const float b, const float g, const float a)
{
    visualization_msgs::msg::Marker marker;
    marker.ns = "Features";
    marker.id = index;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.header.stamp = time;
    marker.header.frame_id = frame_id;
    marker.scale.x = radius * 2.0;
    marker.scale.y = radius * 2.0;
    marker.scale.z = 0.6;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.pose.position.x = static_cast<double>(position.x());
    marker.pose.position.y = static_cast<double>(position.y());
    marker.pose.position.z = marker.scale.z / 2.0;
    marker.pose.orientation.w = 1.0;
    return marker;
}

}  // namespace

Cartographer::Cartographer(const NodeOptions& node_options, const TrajectoryOptions& trajectory_options,
                           const bool collect_metrics)
    : Node("cartographer_node"), node_options_(node_options), trajectory_options_(trajectory_options)
{

    // this->declare_parameter("use_sim_time", false);

    this->map_loaded_ = false;
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node*) {});

    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(node_handle_));
    tf_buffer_.reset(new tf2_ros::Buffer(get_clock(), ::tf2::durationFromSec(kTfBufferCacheTimeInSeconds)));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    absl::MutexLock lock(&mutex_);

    map_builder_bridge_ = std::make_unique<MapBuilderBridge>(node_options_, tf_buffer_);

    // custom_qos_profile_ = rclcpp::QoS(rclcpp::KeepLast(1));
    // custom_qos_profile_ = rclcpp::QoS(1);
    // custom_qos_profile_.keep_last(1);
    // custom_qos_profile_.best_effort();

    if (collect_metrics)
    {
        metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
        carto::metrics::RegisterAllMetrics(metrics_registry_.get());
    }

    submap_list_publisher_ = this->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
        kSubmapListTopic, rclcpp::QoS(rclcpp::KeepLast(1)));
    trajectory_node_list_publisher_ = this->create_publisher<::visualization_msgs::msg::MarkerArray>(
        kTrajectoryNodeListTopic, rclcpp::QoS(rclcpp::KeepLast(1)));
    landmark_poses_list_publisher_ = this->create_publisher<::visualization_msgs::msg::MarkerArray>(
        kLandmarkPosesListTopic, rclcpp::QoS(rclcpp::KeepLast(1)));
    constraint_list_publisher_ = this->create_publisher<::visualization_msgs::msg::MarkerArray>(
        kConstraintListTopic, rclcpp::QoS(rclcpp::KeepLast(1)));
    occupancy_grid_publisher_ =
        this->create_publisher<::nav_msgs::msg::OccupancyGrid>(kOccupancyGridTopic, rclcpp::QoS(rclcpp::KeepLast(1)));

    submap_query_service_ = this->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
        kSubmapQueryServiceName, std::bind(&Cartographer::HandleSubmapQuery, this, _1, _2));
    trajectory_query_service_ = this->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
        kTrajectoryQueryServiceName, std::bind(&Cartographer::HandleTrajectoryQuery, this, _1, _2));
    load_state_service_ = this->create_service<cartographer_ros_msgs::srv::LoadState>(
        kLoadStateServiceName, std::bind(&Cartographer::HandleLoadState, this, _1, _2));
    write_state_service_ = this->create_service<cartographer_ros_msgs::srv::WriteState>(
        kWriteStateServiceName, std::bind(&Cartographer::HandleWriteState, this, _1, _2));
    get_trajectory_states_service_ = this->create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
        kGetTrajectoryStatesServiceName, std::bind(&Cartographer::HandleGetTrajectoryStates, this, _1, _2));
    read_metrics_service_ = this->create_service<cartographer_ros_msgs::srv::ReadMetrics>(
        kReadMetricsServiceName, std::bind(&Cartographer::HandleReadMetrics, this, _1, _2));

    start_localisation_service_ = this->create_service<cartographer_ros_msgs::srv::StartLocalisation>(
        "start_localisation", std::bind(&Cartographer::HandleStartLocalisation, this, _1, _2));
    stop_localisation_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_localisation", std::bind(&Cartographer::HandleStopLocalisation, this, _1, _2));

    pause_localisation_service_ = this->create_service<std_srvs::srv::Trigger>(
        "pause_localisation", std::bind(&Cartographer::HandlePauseLocalisation, this, _1, _2));
    resume_localisation_service_ = this->create_service<std_srvs::srv::Trigger>(
        "resume_localisation", std::bind(&Cartographer::HandleResumeLocalisation, this, _1, _2));

    handle_start_mapping_server_ = this->create_service<cartographer_ros_msgs::srv::StartMapping>(
        "start_mapping", std::bind(&Cartographer::HandleStartMapping, this, _1, _2));
    handle_stop_mapping_server_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_mapping", std::bind(&Cartographer::HandleStopMapping, this, _1, _2));

    // auto infinite_qos = rclcpp::QoS(rclcpp::KeepAll());
    auto latching_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latching_qos.transient_local();
    // auto infinite_qos = rclcpp::QoS(1);
    // infinite_qos.keep_all();
    map_data_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        kMapDataTopic, latching_qos, std::bind(&Cartographer::HandleMapData, this, _1));

    scan_matched_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        kScanMatchedPointCloudTopic, rclcpp::QoS(rclcpp::KeepLast(1)));
    scan_features_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("scan_features", rclcpp::QoS(rclcpp::KeepLast(1)));
    submap_features_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "submap_features", rclcpp::QoS(rclcpp::KeepLast(1)));

    system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_IDLE;
    system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
    system_state_.map_loaded = false;

    system_state_publisher_ =
        this->create_publisher<cartographer_ros_msgs::msg::SystemState>("state", rclcpp::QoS(rclcpp::KeepLast(1)));
    system_state_publisher_->publish(system_state_);
}

Cartographer::~Cartographer()
{
    FinishAllTrajectories();
}

// ::rclcpp::Node::SharedPtr Cartographer::node_handle() { return nh_; }

void Cartographer::Reset()
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

    // wall_timers_.clear();
    CancelTimerCallbacks();
    sensor_samplers_.clear();
    subscribers_.clear();
    trajectories_scheduled_for_finish_.clear();
    paused_timer_.reset();

    map_builder_bridge_->RunFinalOptimization();

    map_builder_bridge_ = std::make_unique<MapBuilderBridge>(node_options_, tf_buffer_);

    if (!map_data_.empty())
    {
        try
        {
            // boost::iostreams::stream<boost::iostreams::array_source> ss(map_data_.c_str(), map_data_.size());
            // map_builder_bridge_->LoadState(ss, true);
            std::istringstream is(map_data_);
            map_builder_bridge_->LoadState(is, true);
        }
        catch (const std::exception& e)
        {
            LOG(ERROR) << "Failed to load map data: " << e.what();
        }
    }

    LOG(INFO) << "Reset complete";
}

void Cartographer::StartTimerCallbacks()
{
    // if (this->timers_initialised_){
    //     CancelTimerCallbacks();
    // }
    CancelTimerCallbacks();

    submap_list_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
                                std::bind(&Cartographer::PublishSubmapList, this));
    trajectory_states_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
                                std::bind(&Cartographer::PublishLocalTrajectoryData, this));
    trajectory_node_list_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
                                std::bind(&Cartographer::PublishTrajectoryNodeList, this));
    landmark_pose_list_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
                                std::bind(&Cartographer::PublishLandmarkPosesList, this));
    constrain_list_timer_ = this->create_wall_timer(std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
                                                    std::bind(&Cartographer::PublishConstraintList, this));
    // this->timers_initialised_ = true;

    // wall_timers_.clear();
    // wall_timers_.push_back(this->create_wall_timer(
    //     std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    //     std::bind(&Cartographer::PublishSubmapList, this)));
    // wall_timers_.push_back(this->create_wall_timer(
    //     std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
    //     [this]() {
    //     PublishLocalTrajectoryData();
    //     }));
    // wall_timers_.push_back(this->create_wall_timer(
    //     std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    //     [this]() {
    //     PublishTrajectoryNodeList();
    //     }));
    // wall_timers_.push_back(this->create_wall_timer(
    //     std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    //     [this]() {
    //     PublishLandmarkPosesList();
    //     }));
    // wall_timers_.push_back(this->create_wall_timer(
    //     std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
    //     [this]() {
    //     PublishConstraintList();
    //     }));
}

void Cartographer::CancelTimerCallbacks()
{
    // if (this->timers_initialised_){
    //     LOG(INFO) << "Canceling timers";
    //     submap_list_timer_->cancel();
    //     trajectory_states_timer_->cancel();
    //     trajectory_node_list_timer_->cancel();
    //     landmark_pose_list_timer_->cancel();
    //     constrain_list_timer_->cancel();
    //     LOG(INFO) << "Timers cancelled";
    // }
    // this->timers_initialised_ = false;
    submap_list_timer_.reset();
    trajectory_states_timer_.reset();
    trajectory_node_list_timer_.reset();
    landmark_pose_list_timer_.reset();
    constrain_list_timer_.reset();
}

void Cartographer::HandleSubmapQuery(const std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Request> request,
                                     std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Response> response)
{
    absl::MutexLock lock(&mutex_);
    map_builder_bridge_->HandleSubmapQuery(request, response);
    // return true;
}

void Cartographer::HandleTrajectoryQuery(
    const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response)
{
    absl::MutexLock lock(&mutex_);
    response->status =
        TrajectoryStateToStatus(request->trajectory_id, {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
                                                         TrajectoryState::FROZEN} /* valid states */);
    if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK)
    {
        LOG(ERROR) << "Can't query trajectory from pose graph: " << response->status.message;
        // return true;
    }
    map_builder_bridge_->HandleTrajectoryQuery(request, response);
    // return true;
}

void Cartographer::PublishSubmapList()
{
    absl::MutexLock lock(&mutex_);

    const auto submaps = map_builder_bridge_->GetSubmapList(this->get_clock()->now());
    submap_list_publisher_->publish(submaps);

    // This is very CPU intensive. Throttle rate
    // Only really needed when mapping
    // There is a bug that causes this to crash Cartographer when submaps are trimmed
    if (system_state_.mode == cartographer_ros_msgs::msg::SystemState::MODE_MAPPING)
    {
        static unsigned int og_throttle = 0;
        if (og_throttle == 0)
        {
            const nav_msgs::msg::OccupancyGrid og_map = map_builder_bridge_->GetOccupancyGridMsg(0.02);
            occupancy_grid_publisher_->publish(og_map);
        }
        og_throttle++;
        if (og_throttle >= 5)
        {
            og_throttle = 0;
        }
    }

    if (submap_features_publisher_->get_subscription_count() > 0)
    {
        const auto submap_data = map_builder_bridge_->map_builder().pose_graph()->GetAllSubmapData();

        visualization_msgs::msg::MarkerArray feature_markers;
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        feature_markers.markers.push_back(delete_all);
        const rclcpp::Time feature_time = this->get_clock()->now();
        for (const auto item : submap_data)
        {
            const auto& sm = dynamic_cast<const cartographer::mapping::Submap2D&>(*item.data.submap);

            const cartographer::transform::Rigid3d gp =
                map_builder_bridge_->map_builder().pose_graph()->GetLocalToGlobalTransform(item.id.trajectory_id);

            for (const auto& f : sm.CircleFeatures())
            {
                const cartographer::transform::Rigid3d::Vector p = gp * f.keypoint.position.cast<double>();
                feature_markers.markers.push_back(
                    MakeCylinder(feature_time, f.fdescriptor.radius, static_cast<int>(feature_markers.markers.size()),
                                 node_options_.map_frame, p.head<2>().cast<float>(), 0.0f, 0.0f, 1.0f, 1.0f));
            }
        }

        submap_features_publisher_->publish(feature_markers);
    }
}

void Cartographer::PublishLocalTrajectoryData()
{
    absl::MutexLock lock(&mutex_);

    const auto local_slam_data = map_builder_bridge_->GetLocalSLAMData();
    const auto global_slam_data = map_builder_bridge_->GetGlobalSLAMData();

    const int trajectory_id = (system_state_.mode == cartographer_ros_msgs::msg::SystemState::MODE_MAPPING) ? 0 : 1;

    auto local_it = local_slam_data.find(trajectory_id);
    auto global_it = global_slam_data.local_to_global.find(trajectory_id);

    if (local_it != local_slam_data.end())
    {
        const auto& trajectory_data = local_it->second;

        const auto constraints = map_builder_bridge_->map_builder().pose_graph()->constraints();
        const long global_constraints = std::count_if(
            constraints.begin(), constraints.end(),
            [&trajectory_id](const auto& c)
            { return (c.node_id.trajectory_id == trajectory_id && c.submap_id.trajectory_id != trajectory_id); });
        const cartographer::transform::Rigid3d local_to_global = (global_it != global_slam_data.local_to_global.end())
                                                                     ? global_it->second
                                                                     : cartographer::transform::Rigid3d::Identity();
        const rclcpp::Time ros_time = ToRos(trajectory_data.time);

        if (scan_matched_point_cloud_publisher_->get_subscription_count() > 0)
        {
            carto::sensor::TimedPointCloud point_cloud;
            point_cloud.reserve(trajectory_data.trajectory_node_data->range_data.returns.size());
            std::transform(trajectory_data.trajectory_node_data->range_data.returns.begin(),
                           trajectory_data.trajectory_node_data->range_data.returns.end(),
                           std::back_inserter(point_cloud),
                           [](const cartographer::sensor::RangefinderPoint& point)
                           { return cartographer::sensor::ToTimedRangefinderPoint(point, 0.f /* time */); });
            scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
                carto::common::ToUniversal(trajectory_data.time), trajectory_options_.tracking_frame,
                carto::sensor::TransformTimedPointCloud(point_cloud, local_to_global.cast<float>())));
        }

        if (scan_features_publisher_->get_subscription_count() > 0)
        {
            visualization_msgs::msg::MarkerArray feature_markers;
            visualization_msgs::msg::Marker delete_all;
            delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
            feature_markers.markers.push_back(delete_all);
            const rclcpp::Time feature_time = ros_time;
            for (const auto& feature : trajectory_data.trajectory_node_data->circle_features)
            {
                feature_markers.markers.push_back(MakeCylinder(
                    feature_time, feature.fdescriptor.radius, static_cast<int>(feature_markers.markers.size()),
                    trajectory_options_.tracking_frame, feature.keypoint.position.head<2>(), 1.0f, 0.0f, 0.0f, 1.0f));
            }

            scan_features_publisher_->publish(feature_markers);
        }

        geometry_msgs::msg::TransformStamped stamped_transform;

        const Rigid3d tracking_in_local =
            carto::transform::Embed3D(carto::transform::Project2D(trajectory_data.local_to_tracking));
        const Rigid3d tracking_in_global = local_to_global * tracking_in_local;

        {
            const Rigid3d global_to_odom = tracking_in_global * trajectory_data.tracking_to_odom;
            stamped_transform.header.stamp = this->get_clock()->now();
            stamped_transform.header.frame_id = node_options_.map_frame;
            stamped_transform.child_frame_id = trajectory_options_.odom_frame;
            stamped_transform.transform = ToGeometryMsgTransform(global_to_odom);
            tf_broadcaster_->sendTransform(stamped_transform);

            system_state_.localisation = stamped_transform;
            system_state_.localisation.header.stamp = ToRos(trajectory_data.time);
        }

        system_state_.number_of_global_constraints = global_constraints;

        if (system_state_.mode == cartographer_ros_msgs::msg::SystemState::MODE_MAPPING)
        {
            system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOCALISED;
        }
        else
        {
            if (global_constraints > 0)
            {
                system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOCALISED;
            }
            else
            {
                system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
            }
        }

        system_state_publisher_->publish(system_state_);
    }
}

void Cartographer::PublishTrajectoryNodeList()
{
    if (trajectory_node_list_publisher_->get_subscription_count() > 0)
    {
        absl::MutexLock lock(&mutex_);
        trajectory_node_list_publisher_->publish(map_builder_bridge_->GetTrajectoryNodeList(this->get_clock()->now()));
    }
}

void Cartographer::PublishLandmarkPosesList()
{
    if (landmark_poses_list_publisher_->get_subscription_count() > 0)
    {
        absl::MutexLock lock(&mutex_);
        landmark_poses_list_publisher_->publish(map_builder_bridge_->GetLandmarkPosesList(this->get_clock()->now()));
    }
}

void Cartographer::PublishConstraintList()
{
    if (constraint_list_publisher_->get_subscription_count() > 0)
    {
        absl::MutexLock lock(&mutex_);
        constraint_list_publisher_->publish(map_builder_bridge_->GetConstraintList(this->get_clock()->now()));
    }
}

int Cartographer::AddTrajectory(const TrajectoryOptions& options)
{
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> expected_sensor_ids =
        ComputeExpectedSensorIds(options);
    const int trajectory_id = map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);

    CHECK(sensor_samplers_.count(trajectory_id) == 0);
    sensor_samplers_.emplace(std::piecewise_construct, std::forward_as_tuple(trajectory_id),
                             std::forward_as_tuple(options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
                                                   options.fixed_frame_pose_sampling_ratio,
                                                   options.landmarks_sampling_ratio));

    for (const std::string& topic : ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans))
    {
        LOG(INFO) << "Subscribing to topic " << topic;
        std::function<void(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)> lsm_fcn =
            std::bind(&Cartographer::HandleLaserScanMessage, this, trajectory_id, topic, _1);
        subscribers_[trajectory_id].push_back(
            {this->create_subscription<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS(), lsm_fcn), topic});
        // subscribers_[trajectory_id].push_back(SubscribeWithHandler<sensor_msgs::msg::LaserScan>(&Cartographer::HandleLaserScanMessage,
        // trajectory_id, topic, node_handle_, this, custom_qos_profile_));
    }

    for (const std::string& topic :
         ComputeRepeatedTopicNames(kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans))
    {
        std::function<void(const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg)> melsm_fcn =
            std::bind(&Cartographer::HandleMultiEchoLaserScanMessage, this, trajectory_id, topic, _1);
        subscribers_[trajectory_id].push_back({this->create_subscription<sensor_msgs::msg::MultiEchoLaserScan>(
                                                   topic, rclcpp::QoS(rclcpp::KeepLast(1)), melsm_fcn),
                                               topic});
        // subscribers_[trajectory_id].push_back(
        //     {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(&Cartographer::HandleMultiEchoLaserScanMessage,
        //                                                            trajectory_id, topic, nh_, this,
        //                                                            custom_qos_profile_),
        //      topic});
    }

    for (const std::string& topic : ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds))
    {
        std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)> pcm_fcn =
            std::bind(&Cartographer::HandlePointCloud2Message, this, trajectory_id, topic, _1);
        subscribers_[trajectory_id].push_back(
            {this->create_subscription<sensor_msgs::msg::PointCloud2>(topic, rclcpp::QoS(rclcpp::KeepLast(1)), pcm_fcn),
             topic});
        // subscribers_[trajectory_id].push_back({SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
        //                                            &Cartographer::HandlePointCloud2Message, trajectory_id, topic,
        //                                            nh_, this, custom_qos_profile_),
        //                                        topic});
    }

    if (options.use_odometry)
    {
        std::function<void(const nav_msgs::msg::Odometry::ConstSharedPtr msg)> om_fcn =
            std::bind(&Cartographer::HandleOdometryMessage, this, trajectory_id, kOdometryTopic, _1);
        subscribers_[trajectory_id].push_back({this->create_subscription<nav_msgs::msg::Odometry>(
                                                   kOdometryTopic, rclcpp::QoS(rclcpp::KeepLast(1)), om_fcn),
                                               kOdometryTopic});
        // subscribers_[trajectory_id].push_back(
        //     {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Cartographer::HandleOdometryMessage, trajectory_id,
        //     kOdometryTopic, nh_,
        //                                               this, custom_qos_profile_),
        //      kOdometryTopic});
    }

    if (options.use_nav_sat)
    {
        std::function<void(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)> nsfm_fcn =
            std::bind(&Cartographer::HandleNavSatFixMessage, this, trajectory_id, kNavSatFixTopic, _1);
        subscribers_[trajectory_id].push_back({this->create_subscription<sensor_msgs::msg::NavSatFix>(
                                                   kNavSatFixTopic, rclcpp::QoS(rclcpp::KeepLast(1)), nsfm_fcn),
                                               kNavSatFixTopic});
        // subscribers_[trajectory_id].push_back(
        //     {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(&Cartographer::HandleNavSatFixMessage, trajectory_id,
        //     kNavSatFixTopic,
        //                                                   nh_, this, custom_qos_profile_),
        //      kNavSatFixTopic});
    }

    if (options.use_landmarks)
    {
        std::function<void(const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr msg)> lm_fcn =
            std::bind(&Cartographer::HandleLandmarkMessage, this, trajectory_id, kLandmarkTopic, _1);
        subscribers_[trajectory_id].push_back({this->create_subscription<cartographer_ros_msgs::msg::LandmarkList>(
                                                   kLandmarkTopic, rclcpp::QoS(rclcpp::KeepLast(1)), lm_fcn),
                                               kLandmarkTopic});
        // subscribers_[trajectory_id].push_back(
        //     {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(&Cartographer::HandleLandmarkMessage,
        //     trajectory_id,
        //                                                                kLandmarkTopic, nh_, this,
        //                                                                custom_qos_profile_),
        //      kLandmarkTopic});
    }

    // wall_timers_.push_back(
    //     nh_.createWallTimer(this->create_wall_timer(
    //     std::chrono::milliseconds(int(kTopicMismatchCheckDelaySec * 1000)),
    //     [this]() {
    //     MaybeWarnAboutTopicMismatch();
    //     });

    return trajectory_id;
}

int Cartographer::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
    const TrajectoryOptions& options)
{
    absl::MutexLock lock(&mutex_);
    const int trajectory_id = map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);

    CHECK(sensor_samplers_.count(trajectory_id) == 0);
    sensor_samplers_.emplace(std::piecewise_construct, std::forward_as_tuple(trajectory_id),
                             std::forward_as_tuple(options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
                                                   options.fixed_frame_pose_sampling_ratio,
                                                   options.landmarks_sampling_ratio));
    return trajectory_id;
}

cartographer_ros_msgs::msg::StatusResponse
    Cartographer::TrajectoryStateToStatus(const int trajectory_id, const std::set<TrajectoryState>& valid_states)
{
    const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
    cartographer_ros_msgs::msg::StatusResponse status_response;

    const auto it = trajectory_states.find(trajectory_id);
    if (it == trajectory_states.end())
    {
        // status_response.message = absl::StrCat("Trajectory ", trajectory_id, " doesn't exist.");
        status_response.message = "Trajectory " + std::to_string(trajectory_id) + " doesn't exist.";
        status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
        return status_response;
    }

    // status_response.message =
    //     absl::StrCat("Trajectory ", trajectory_id, " is in '", TrajectoryStateToString(it->second), "' state.");
    status_response.message =
        "Trajectory " + std::to_string(trajectory_id) + " is in '" + TrajectoryStateToString(it->second) + "' state.";
    status_response.code = valid_states.count(it->second) ? cartographer_ros_msgs::msg::StatusCode::OK
                                                          : cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    return status_response;
}

cartographer_ros_msgs::msg::StatusResponse Cartographer::FinishTrajectoryUnderLock(const int trajectory_id)
{
    cartographer_ros_msgs::msg::StatusResponse status_response;
    if (trajectories_scheduled_for_finish_.count(trajectory_id))
    {
        // status_response.message = absl::StrCat("Trajectory ", trajectory_id, " already pending to finish.");
        status_response.message = "Trajectory " + std::to_string(trajectory_id) + " already pending to finish.";
        status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
        LOG(INFO) << status_response.message;
        return status_response;
    }

    // First, check if we can actually finish the trajectory.
    status_response = TrajectoryStateToStatus(trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
    if (status_response.code != cartographer_ros_msgs::msg::StatusCode::OK)
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
            entry.subscriber.reset();
            LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
            // entry.reset(); // Reseting subcription's SharedPtr
            // LOG(INFO) << "Shutdown the subscriber of [" << entry.get()->get_topic_name() << "]";
        }
        CHECK_EQ(subscribers_.erase(trajectory_id), 1);
    }
    map_builder_bridge_->FinishTrajectory(trajectory_id);
    trajectories_scheduled_for_finish_.emplace(trajectory_id);
    // status_response.message = absl::StrCat("Finished trajectory ", trajectory_id, ".");
    status_response.message = "Finished trajectory " + std::to_string(trajectory_id) + ".";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
    return status_response;
}

void Cartographer::HandleLoadState(const std::shared_ptr<cartographer_ros_msgs::srv::LoadState::Request> request,
                                   std::shared_ptr<cartographer_ros_msgs::srv::LoadState::Response> response)
{
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Loading incoming map data (" << request->pbstream_data.size() << ")...";

    map_data_ =
        std::string(reinterpret_cast<const char*>(request->pbstream_data.data()), request->pbstream_data.size());

    Reset();

    system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_IDLE;
    system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
    system_state_.map_loaded = !map_data_.empty();
    system_state_.number_of_global_constraints = 0;
    system_state_publisher_->publish(system_state_);

    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
}

void Cartographer::HandleGetTrajectoryStates(
    const std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Response> response)
{
    (void)request;
    using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
    absl::MutexLock lock(&mutex_);
    response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
    response->trajectory_states.header.stamp = this->get_clock()->now();
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        response->trajectory_states.trajectory_id.push_back(entry.first);
        switch (entry.second)
        {
            case TrajectoryState::ACTIVE:
                response->trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE);
                break;
            case TrajectoryState::FINISHED:
                response->trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::msg::TrajectoryStates::FINISHED);
                break;
            case TrajectoryState::FROZEN:
                response->trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::msg::TrajectoryStates::FROZEN);
                break;
            case TrajectoryState::DELETED:
                response->trajectory_states.trajectory_state.push_back(
                    ::cartographer_ros_msgs::msg::TrajectoryStates::DELETED);
                break;
        }
    }
}

void Cartographer::HandleWriteState(const std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Request> request,
                                    std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Response> response)
{
    absl::MutexLock lock(&mutex_);

    nav_msgs::msg::OccupancyGrid og_map = map_builder_bridge_->GetOccupancyGridMsg(request->resolution);

    const auto local_slam_data = map_builder_bridge_->GetLocalSLAMData();
    const auto global_slam_data = map_builder_bridge_->GetGlobalSLAMData();

    const int trajectory_id = (system_state_.mode == cartographer_ros_msgs::msg::SystemState::MODE_MAPPING) ? 0 : 1;

    auto local_it = local_slam_data.find(trajectory_id);
    auto global_it = global_slam_data.local_to_global.find(trajectory_id);

    CHECK(local_it != local_slam_data.end());

    {
        const auto& trajectory_data = local_it->second;
        const Rigid3d tracking_in_local =
            carto::transform::Embed3D(carto::transform::Project2D(trajectory_data.local_to_tracking));
        const cartographer::transform::Rigid3d local_to_global = (global_it != global_slam_data.local_to_global.end())
                                                                     ? global_it->second
                                                                     : cartographer::transform::Rigid3d::Identity();
        const Rigid3d tracking_in_global = local_to_global * tracking_in_local;
        response->trajectory_id.push_back(local_it->first);
        response->pose.push_back(ToGeometryMsgPose(tracking_in_global));
    }

    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        if (entry.second == TrajectoryState::ACTIVE)
        {
            CHECK_EQ(FinishTrajectoryUnderLock(entry.first).code, cartographer_ros_msgs::msg::StatusCode::OK);
        }
    }

    map_builder_bridge_->RunFinalOptimization();

    response->map_info = og_map.info;

    std::stringstream ss;
    if (map_builder_bridge_->SerializeState(ss, false))
    {
        ss.seekp(0);
        ss >> std::noskipws;
        std::copy(std::istream_iterator<uint8_t>(ss), std::istream_iterator<uint8_t>(),
                  std::back_inserter(response->pbstream_data));

        {
            response->occupancy_grid.format = "png";

            png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
            png_infop info = png_create_info_struct(png);

            if (setjmp(png_jmpbuf(png)))
                abort();

            auto user_write_fn = [](png_structp png, png_bytep data, png_size_t length)
            {
                auto v = (sensor_msgs::msg::CompressedImage::_data_type*)png_get_io_ptr(png);
                for (unsigned int i = 0; i < length; ++i)
                {
                    v->push_back(*data++);
                }
            };

            png_set_write_fn(png, (void*)&response->occupancy_grid.data, user_write_fn, nullptr);

            png_set_IHDR(png, info, og_map.info.width, og_map.info.height, 8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                         PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

            png_color_8 sig_bit;
            sig_bit.gray = 8;
            png_set_sBIT(png, info, &sig_bit);

            png_write_info(png, info);

            const int bytes_per_pixel = 1;

            if (og_map.info.height > PNG_SIZE_MAX / (og_map.info.width * bytes_per_pixel))
                png_error(png, "Image data buffer would be too large");

            png_bytep* row_pointers = new png_bytep[og_map.info.height];

            if (og_map.info.height > PNG_UINT_32_MAX / (sizeof(png_bytep)))
                png_error(png, "Image is too tall to process in memory");

            for (png_uint_32 k = 0; k < og_map.info.height; k++)
                row_pointers[k] =
                    reinterpret_cast<unsigned char*>(&og_map.data[0]) + k * og_map.info.width * bytes_per_pixel;

            png_write_image(png, row_pointers);

            png_write_end(png, info);

            png_destroy_write_struct(&png, &info);
        }

        response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
        // response->status.message = absl::StrCat("Success");
        response->status.message = "Success";
    }
    else
    {
        response->status.code = cartographer_ros_msgs::msg::StatusCode::CANCELLED;
        // response->status.message = absl::StrCat("Failed");
        response->status.message = "Failed";
    }

    Reset();

    system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_IDLE;
    system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
    system_state_.map_loaded = !map_data_.empty();
    system_state_.number_of_global_constraints = 0;
    system_state_publisher_->publish(system_state_);

    // return true;
}

void Cartographer::HandleReadMetrics(const std::shared_ptr<cartographer_ros_msgs::srv::ReadMetrics::Request> request,
                                     std::shared_ptr<cartographer_ros_msgs::srv::ReadMetrics::Response> response)
{
    (void)request;
    absl::MutexLock lock(&mutex_);
    response->timestamp = this->get_clock()->now();
    if (!metrics_registry_)
    {
        response->status.code = cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
        response->status.message = "Collection of runtime metrics is not activated.";
        // return true;
    }
    metrics_registry_->ReadMetrics(response);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message = "Successfully read metrics.";
    // return true;
}

void Cartographer::HandleStartLocalisation(
    const std::shared_ptr<cartographer_ros_msgs::srv::StartLocalisation::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::StartLocalisation::Response> response)
{
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to start localisation";

    if (map_data_.empty())
    {
        response->status.message = "No map data loaded";
        response->status.code = cartographer_ros_msgs::msg::StatusCode::CANCELLED;
    }
    else
    {
        Reset();

        TrajectoryOptions trajectory_options = trajectory_options_;

        auto trimmer = trajectory_options.trajectory_builder_options.mutable_pure_localization_trimmer();
        trimmer->set_max_submaps_to_keep(4);

        if (request->use_initial_pose)
        {
            const auto pose = ToRigid3d(request->initial_pose);
            if (!pose.IsValid())
            {
                response->status.message = "Invalid pose argument. Orientation quaternion must be normalized.";
                LOG(ERROR) << response->status.message;
                response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
                // return true;
            }

            // Check if the requested trajectory for the relative initial pose exists.
            response->status = TrajectoryStateToStatus(
                request->relative_to_trajectory_id,
                {TrajectoryState::ACTIVE, TrajectoryState::FROZEN, TrajectoryState::FINISHED} /* valid states */);
            if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK)
            {
                LOG(ERROR) << "Can't start a trajectory with initial pose: " << response->status.message;
                // return true;
            }

            const auto& node = map_builder_bridge_->map_builder().pose_graph()->GetTrajectoryNodes().at(
                cartographer::mapping::NodeId{request->relative_to_trajectory_id, 0});

            ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
            initial_trajectory_pose.set_to_trajectory_id(request->relative_to_trajectory_id);
            *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(pose);
            initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(node.time()));
            *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;
        }

        const int trajectory_id = AddTrajectory(trajectory_options);
        CHECK(trajectory_id == 1);

        response->status.message = "Success.";
        response->trajectory_id = trajectory_id;
        response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;

        system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_LOCALISING;
        system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
        system_state_.map_loaded = !map_data_.empty();
        system_state_.number_of_global_constraints = 0;
        system_state_publisher_->publish(system_state_);

        LOG(INFO) << "StartLocalisation: Starting timers.";
        StartTimerCallbacks();
    }

    LOG(INFO) << "Localization started.";
    // return true;
}

void Cartographer::HandlePauseLocalisation(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to pause localisation";

    if (system_state_.mode != cartographer_ros_msgs::msg::SystemState::MODE_LOCALISING)
    {
        res->message = "Not currently localising!";
        res->success = false;
    }
    else
    {
        const auto local_slam_data = map_builder_bridge_->GetLocalSLAMData();
        const auto global_slam_data = map_builder_bridge_->GetGlobalSLAMData();

        const int trajectory_id = (system_state_.mode == cartographer_ros_msgs::msg::SystemState::MODE_MAPPING) ? 0 : 1;

        auto local_it = local_slam_data.find(trajectory_id);
        auto global_it = global_slam_data.local_to_global.find(trajectory_id);

        CHECK(local_it != local_slam_data.end());

        const auto& trajectory_data = local_it->second;

        const Rigid3d tracking_in_local =
            carto::transform::Embed3D(carto::transform::Project2D(trajectory_data.local_to_tracking));
        const cartographer::transform::Rigid3d local_to_global = (global_it != global_slam_data.local_to_global.end())
                                                                     ? global_it->second
                                                                     : cartographer::transform::Rigid3d::Identity();
        paused_tracking_in_global_ = local_to_global * tracking_in_local;
        paused_global_to_odom_ = paused_tracking_in_global_ * trajectory_data.tracking_to_odom;

        Reset();

        system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_LOCALISING;
        system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::PAUSED;
        system_state_.map_loaded = !map_data_.empty();
        system_state_.number_of_global_constraints = 0;
        system_state_publisher_->publish(system_state_);

        paused_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
                                    std::bind(&Cartographer::PausedTimer, this));

        res->success = true;
    }

    // return true;
}

void Cartographer::PausedTimer()
{
    absl::MutexLock lock(&mutex_);
    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = this->get_clock()->now();
    stamped_transform.header.frame_id = node_options_.map_frame;
    stamped_transform.child_frame_id = trajectory_options_.odom_frame;
    stamped_transform.transform = ToGeometryMsgTransform(paused_global_to_odom_);
    tf_broadcaster_->sendTransform(stamped_transform);
}

void Cartographer::HandleResumeLocalisation(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to resume localisation";

    if (system_state_.mode != cartographer_ros_msgs::msg::SystemState::MODE_LOCALISING ||
        system_state_.localisation_status != cartographer_ros_msgs::msg::SystemState::PAUSED)
    {
        res->message = "Not currently paused!";
        res->success = false;
    }
    else
    {
        Reset();

        TrajectoryOptions trajectory_options = trajectory_options_;

        auto trimmer = trajectory_options.trajectory_builder_options.mutable_pure_localization_trimmer();
        trimmer->set_max_submaps_to_keep(4);

        // Check if the requested trajectory for the relative initial pose exists
        const auto status = TrajectoryStateToStatus(
            0, {TrajectoryState::ACTIVE, TrajectoryState::FROZEN, TrajectoryState::FINISHED} /* valid states */);
        CHECK(status.code == cartographer_ros_msgs::msg::StatusCode::OK);

        const auto& node = map_builder_bridge_->map_builder().pose_graph()->GetTrajectoryNodes().at(
            cartographer::mapping::NodeId{0, 0});

        ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
        initial_trajectory_pose.set_to_trajectory_id(0);
        *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(paused_tracking_in_global_);
        initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(node.time()));

        *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;

        const int trajectory_id = AddTrajectory(trajectory_options);
        CHECK(trajectory_id == 1);

        system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_LOCALISING;
        system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
        system_state_.map_loaded = !map_data_.empty();
        system_state_.number_of_global_constraints = 0;
        system_state_publisher_->publish(system_state_);

        StartTimerCallbacks();

        res->success = true;
    }

    // return true;
}

void Cartographer::HandleStopLocalisation(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to stop localisation";

    if (system_state_.mode != cartographer_ros_msgs::msg::SystemState::MODE_LOCALISING)
    {
        res->message = "Not currently localising!";
        res->success = false;
    }
    else
    {
        Reset();

        system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_IDLE;
        system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
        system_state_.map_loaded = !map_data_.empty();
        system_state_.number_of_global_constraints = 0;
        system_state_publisher_->publish(system_state_);
        res->success = true;
    }

    // return true;
}

void Cartographer::HandleStartMapping(const std::shared_ptr<cartographer_ros_msgs::srv::StartMapping::Request> req,
                                      std::shared_ptr<cartographer_ros_msgs::srv::StartMapping::Response> response)
{
    (void)req;
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to start mapping";

    map_data_.clear();

    Reset();

    response->status.message = "Success.";
    response->trajectory_id = AddTrajectory(trajectory_options_);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;

    StartTimerCallbacks();

    system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_MAPPING;
    system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
    system_state_.map_loaded = !map_data_.empty();
    system_state_.number_of_global_constraints = 0;
    system_state_publisher_->publish(system_state_);

    // return true;
}

void Cartographer::HandleStopMapping(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    absl::MutexLock lock(&mutex_);

    LOG(INFO) << "Request to stop mapping";

    if (system_state_.mode != cartographer_ros_msgs::msg::SystemState::MODE_MAPPING)
    {
        res->message = "Not currently mapping!";
        res->success = false;
    }
    else
    {
        Reset();

        system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_IDLE;
        system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
        system_state_.map_loaded = !map_data_.empty();
        system_state_.number_of_global_constraints = 0;
        system_state_publisher_->publish(system_state_);

        res->success = true;
    }

    // return true;
}

void Cartographer::HandleMapData(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    absl::MutexLock lock(&mutex_);

    if (this->map_loaded_)
        return;

    LOG(INFO) << "Loading incoming map data (" << msg->data.size() << ")...";

    map_data_ = std::string(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

    Reset();

    system_state_.mode = cartographer_ros_msgs::msg::SystemState::MODE_IDLE;
    system_state_.localisation_status = cartographer_ros_msgs::msg::SystemState::LOST;
    system_state_.map_loaded = !map_data_.empty();
    system_state_.number_of_global_constraints = 0;
    system_state_publisher_->publish(system_state_);

    this->map_loaded_ = true;
}

void Cartographer::FinishAllTrajectories()
{
    absl::MutexLock lock(&mutex_);
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates())
    {
        if (entry.second == TrajectoryState::ACTIVE)
        {
            const int trajectory_id = entry.first;
            CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code, cartographer_ros_msgs::msg::StatusCode::OK);
        }
    }
}

bool Cartographer::FinishTrajectory(const int trajectory_id)
{
    absl::MutexLock lock(&mutex_);
    return FinishTrajectoryUnderLock(trajectory_id).code == cartographer_ros_msgs::msg::StatusCode::OK;
}

void Cartographer::RunFinalOptimization()
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

void Cartographer::HandleOdometryMessage(const int trajectory_id, const std::string& sensor_id,
                                         const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // LOG(INFO) << "Received odometry message";
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse())
    {
        return;
    }
    auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
    sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Cartographer::HandleNavSatFixMessage(const int trajectory_id, const std::string& sensor_id,
                                          const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleNavSatFixMessage(sensor_id, msg);
}

void Cartographer::HandleLandmarkMessage(const int trajectory_id, const std::string& sensor_id,
                                         const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleLandmarkMessage(sensor_id, msg);
}

void Cartographer::HandleLaserScanMessage(const int trajectory_id, const std::string& sensor_id,
                                          const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    // LOG(INFO) << "Received laserscan message";
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleLaserScanMessage(sensor_id, msg);
}

void Cartographer::HandleMultiEchoLaserScanMessage(const int trajectory_id, const std::string& sensor_id,
                                                   const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Cartographer::HandlePointCloud2Message(const int trajectory_id, const std::string& sensor_id,
                                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    absl::MutexLock lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)->HandlePointCloud2Message(sensor_id, msg);
}

void Cartographer::SerializeState(const std::string& filename, const bool include_unfinished_submaps)
{
    absl::MutexLock lock(&mutex_);
    std::ofstream file(filename);
    CHECK(map_builder_bridge_->SerializeState(file, include_unfinished_submaps)) << "Could not write state.";
}

void Cartographer::LoadState(const std::string& state_filename, const bool load_frozen_state)
{
    absl::MutexLock lock(&mutex_);
    const std::string suffix = ".pbstream";
    CHECK_EQ(state_filename.substr(std::max<int>(state_filename.size() - suffix.size(), 0)), suffix)
        << "The file containing the state to be loaded must be a .pbstream file.";
    std::ifstream file(state_filename);
    map_builder_bridge_->LoadState(file, load_frozen_state);
}

// Not sure if this functionality exists in ROS2

// void Cartographer::MaybeWarnAboutTopicMismatch()
// {
//     ::ros::master::V_TopicInfo ros_topics;
//     ::ros::master::getTopics(ros_topics);
//     std::set<std::string> published_topics;
//     std::stringstream published_topics_string;
//     for (const auto& it : ros_topics)
//     {
//         std::string resolved_topic = nh_.resolveName(it.name, false);
//         published_topics.insert(resolved_topic);
//         published_topics_string << resolved_topic << ",";
//     }
//     bool print_topics = false;
//     for (const auto& entry : subscribers_)
//     {
//         int trajectory_id = entry.first;
//         for (const auto& subscriber : entry.second)
//         {
//             std::string resolved_topic = nh_.resolveName(subscriber.topic);
//             if (published_topics.count(resolved_topic) == 0)
//             {
//                 LOG(WARNING) << "Expected topic \"" << subscriber.topic << "\" (trajectory " << trajectory_id << ")"
//                              << " (resolved topic \"" << resolved_topic << "\")"
//                              << " but no publisher is currently active.";
//                 print_topics = true;
//             }
//         }
//     }
//     if (print_topics)
//     {
//         LOG(WARNING) << "Currently available topics are: " << published_topics_string.str();
//     }
// }

}  // namespace cartographer_ros

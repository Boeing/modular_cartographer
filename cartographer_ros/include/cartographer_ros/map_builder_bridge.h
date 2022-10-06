#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <cartographer/mapping/map_builder.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <rclcpp/time.hpp>

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif
#include "visualization_msgs/msg/marker_array.hpp"

namespace cartographer_ros
{

class MapBuilderBridge
{
  public:
    struct LocalSLAMData
    {
        ::cartographer::common::Time time;
        ::cartographer::transform::Rigid3d local_to_tracking;
        ::cartographer::transform::Rigid3d tracking_to_odom;
        std::shared_ptr<const ::cartographer::mapping::TrajectoryNode::Data> trajectory_node_data;
    };

    struct GlobalSLAMData
    {
        long unsigned int count = 0;
        ::cartographer::common::Time time;
        std::unordered_map<int, cartographer::transform::Rigid3d> local_to_global;
    };

    MapBuilderBridge(const NodeOptions& node_options, const std::shared_ptr<const tf2_ros::Buffer>& tf_buffer);

    ~MapBuilderBridge();

    MapBuilderBridge(const MapBuilderBridge&) = delete;
    MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

    void LoadState(std::istream& stream, bool load_frozen_state);

    int AddTrajectory(
        const std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
        const TrajectoryOptions& trajectory_options);

    void FinishTrajectory(int trajectory_id);
    void DeleteTrajectory(int trajectory_id);

    void RunFinalOptimization();

    bool SerializeState(std::ostream& stream, const bool include_unfinished_submaps);

    void HandleSubmapQuery(const std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Request> request,
      std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Response> response);
    void HandleTrajectoryQuery(const std::shared_ptr<::cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
                               std::shared_ptr<::cartographer_ros_msgs::srv::TrajectoryQuery::Response> response);

    std::map<int /* trajectory_id */, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
        GetTrajectoryStates();
    cartographer_ros_msgs::msg::SubmapList GetSubmapList(::rclcpp::Time node_time);

    std::unordered_map<int, LocalSLAMData> GetLocalSLAMData() const LOCKS_EXCLUDED(mutex_);
    GlobalSLAMData GetGlobalSLAMData() const LOCKS_EXCLUDED(mutex_);

    visualization_msgs::msg::MarkerArray GetTrajectoryNodeList(::rclcpp::Time node_time);
    visualization_msgs::msg::MarkerArray GetLandmarkPosesList(::rclcpp::Time node_time);
    visualization_msgs::msg::MarkerArray GetConstraintList(::rclcpp::Time node_time);

    nav_msgs::msg::OccupancyGrid GetOccupancyGridMsg(const double resolution);

    SensorBridge* sensor_bridge(int trajectory_id);
    cartographer::mapping::MapBuilder& map_builder()
    {
        return map_builder_;
    };

  private:
    void OnLocalSlamResult(
        const int trajectory_id, const ::cartographer::common::Time time,
        const ::cartographer::transform::Rigid3d local_pose, const ::cartographer::transform::Rigid3d odom,
        const std::unique_ptr<const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult>
            insertion_result) LOCKS_EXCLUDED(mutex_);

    void OnGlobalSlamOptimization(const std::map<int, ::cartographer::mapping::SubmapId>& submap,
                                  const std::map<int, ::cartographer::mapping::NodeId>& node) LOCKS_EXCLUDED(mutex_);

    mutable absl::Mutex mutex_;
    const NodeOptions node_options_;

    // SLAM Data
    std::unordered_map<int, LocalSLAMData> local_slam_data_ GUARDED_BY(mutex_);
    GlobalSLAMData global_slam_data_ GUARDED_BY(mutex_);

    cartographer::mapping::MapBuilder map_builder_;
    const std::shared_ptr<const tf2_ros::Buffer> tf_buffer_;

    std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

    // These are keyed with 'trajectory_id'.
    std::unordered_map<int, TrajectoryOptions> trajectory_options_;
    std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
    std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

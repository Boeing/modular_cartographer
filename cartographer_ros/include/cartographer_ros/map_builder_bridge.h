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
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros
{

class MapBuilderBridge
{
  public:
    struct LocalSLAMData
    {
        ::cartographer::common::Time time;
        ::cartographer::transform::Rigid3d local_to_tracking;
        std::shared_ptr<cartographer::transform::Rigid3d> tracking_to_odom;
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

    void HandleSubmapQuery(cartographer_ros_msgs::SubmapQuery::Request& request,
                           cartographer_ros_msgs::SubmapQuery::Response& response);
    void HandleTrajectoryQuery(cartographer_ros_msgs::TrajectoryQuery::Request& request,
                               cartographer_ros_msgs::TrajectoryQuery::Response& response);

    std::map<int /* trajectory_id */, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
        GetTrajectoryStates();
    cartographer_ros_msgs::SubmapList GetSubmapList();

    std::unordered_map<int, LocalSLAMData> GetLocalSLAMData() const LOCKS_EXCLUDED(mutex_);
    GlobalSLAMData GetGlobalSLAMData() const LOCKS_EXCLUDED(mutex_);

    visualization_msgs::MarkerArray GetTrajectoryNodeList();
    visualization_msgs::MarkerArray GetLandmarkPosesList();
    visualization_msgs::MarkerArray GetConstraintList();

    nav_msgs::OccupancyGrid GetOccupancyGridMsg(const double resolution);

    SensorBridge* sensor_bridge(int trajectory_id);
    cartographer::mapping::MapBuilder& map_builder()
    {
        return map_builder_;
    };

  private:
    void OnLocalSlamResult(
        const int trajectory_id, const ::cartographer::common::Time time,
        const ::cartographer::transform::Rigid3d local_pose,
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

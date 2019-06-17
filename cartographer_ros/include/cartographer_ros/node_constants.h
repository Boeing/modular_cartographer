#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_CONSTANTS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_CONSTANTS_H

#include <set>
#include <string>
#include <vector>

#include <cartographer/mapping/trajectory_builder_interface.h>

#include <cartographer_ros/node_options.h>
#include <cartographer_ros/trajectory_options.h>

namespace cartographer_ros
{

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kNavSatFixTopic[] = "fix";
constexpr char kLandmarkTopic[] = "landmark";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kOccupancyGridTopic[] = "occupancy_grid";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kTrajectoryQueryServiceName[] = "trajectory_query";
constexpr char kLoadStateServiceName[] = "load_state";
constexpr char kWriteStateServiceName[] = "write_state";
constexpr char kGetTrajectoryStatesServiceName[] = "get_trajectory_states";
constexpr char kReadMetricsServiceName[] = "read_metrics";
constexpr char kMapDataTopic[] = "map_data";
constexpr char kTrajectoryNodeListTopic[] = "trajectory_node_list";
constexpr char kLandmarkPosesListTopic[] = "landmark_poses_list";
constexpr char kConstraintListTopic[] = "constraint_list";
constexpr double kConstraintPublishPeriodSec = 0.5;
constexpr double kTopicMismatchCheckDelaySec = 3.0;

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// For multiple topics adds numbers to the topic name and returns the list.
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic, int num_topics);

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    ComputeExpectedSensorIds(const TrajectoryOptions& trajectory_options, const NodeOptions& node_options);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_CONSTANTS_H

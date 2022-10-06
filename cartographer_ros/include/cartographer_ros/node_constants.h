#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_CONSTANTS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_CONSTANTS_H

#include <cartographer/mapping/trajectory_builder_interface.h>
#include <cartographer_ros/trajectory_options.h>

#include <set>
#include <string>
#include <vector>

namespace cartographer_ros
{

// Default topic names; expected to be remapped as needed.
const std::string kLaserScanTopic = "scan";
const std::string kMultiEchoLaserScanTopic = "echoes";
const std::string kPointCloud2Topic = "points2";
const std::string kImuTopic = "imu";
const std::string kOdometryTopic = "odom";
const std::string kNavSatFixTopic = "fix";
const std::string kLandmarkTopic = "landmark";
const std::string kFinishTrajectoryServiceName = "finish_trajectory";
const std::string kOccupancyGridTopic = "occupancy_grid";
const std::string kScanMatchedPointCloudTopic = "scan_matched_points2";
const std::string kSubmapListTopic = "submap_list";
const std::string kSubmapQueryServiceName = "submap_query";
const std::string kTrajectoryQueryServiceName = "trajectory_query";
const std::string kLoadStateServiceName = "load_state";
const std::string kWriteStateServiceName = "write_state";
const std::string kGetTrajectoryStatesServiceName = "get_trajectory_states";
const std::string kReadMetricsServiceName = "read_metrics";
const std::string kMapDataTopic = "map_data";
const std::string kTrajectoryNodeListTopic = "trajectory_node_list";
const std::string kLandmarkPosesListTopic = "landmark_poses_list";
const std::string kConstraintListTopic = "constraint_list";
const double kConstraintPublishPeriodSec = 0.5;
const double kTopicMismatchCheckDelaySec = 3.0;

const int kInfiniteSubscriberQueueSize = 0;

// For multiple topics adds numbers to the topic name and returns the list.
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic, int num_topics);

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    ComputeExpectedSensorIds(const TrajectoryOptions& trajectory_options);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_CONSTANTS_H

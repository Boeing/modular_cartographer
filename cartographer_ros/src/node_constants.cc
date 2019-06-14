#include <cartographer_ros/node_constants.h>

#include "glog/logging.h"

namespace cartographer_ros
{

std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic, const int num_topics)
{
    CHECK_GE(num_topics, 0);
    if (num_topics == 1)
    {
        return {topic};
    }
    std::vector<std::string> topics;
    topics.reserve(num_topics);
    for (int i = 0; i < num_topics; ++i)
    {
        topics.emplace_back(topic + "_" + std::to_string(i + 1));
    }
    return topics;
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> ComputeExpectedSensorIds(const TrajectoryOptions& trajectory_options, const NodeOptions& node_options)
{
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> expected_topics;
    // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
    for (const std::string& topic : ComputeRepeatedTopicNames(kLaserScanTopic, trajectory_options.num_laser_scans))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(kMultiEchoLaserScanTopic, trajectory_options.num_multi_echo_laser_scans))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(kPointCloud2Topic, trajectory_options.num_point_clouds))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
    // required.
    if (node_options.map_builder_options.use_trajectory_builder_3d() ||
        (node_options.map_builder_options.use_trajectory_builder_2d() &&
         trajectory_options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data()))
    {
        expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
    }
    // Odometry is optional.
    if (trajectory_options.use_odometry)
    {
        expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
    }
    // NavSatFix is optional.
    if (trajectory_options.use_nav_sat)
    {
        expected_topics.insert(SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
    }
    // Landmark is optional.
    if (trajectory_options.use_landmarks)
    {
        expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    }
    return expected_topics;
}

}  // namespace cartographer_ros

/*
 * Copyright 2017 The Cartographer Authors
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

#include <errno.h>
#include <sys/resource.h>
#include <time.h>

#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <string>
#include <vector>

// #include "absl/strings/str_split.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/playable_bag.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
// #include "ros/callback_queue.h"
// #include "ros/ros.h"
#include <rclcpp/rclcpp.hpp>

#include "rclcpp/exceptions.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basenames, "",
              "Comma-separated list of basenames, i.e. not containing any "
              "directory prefix, of the configuration files for each trajectory. "
              "The first configuration file will be used for node options. "
              "If less configuration files are specified than trajectories, the "
              "first file will be used the remaining trajectories.");
DEFINE_string(bag_filenames, "",
              "Comma-separated list of bags to process. One bag per trajectory. "
              "Any combination of simultaneous and sequential bags is supported.");
DEFINE_string(urdf_filenames, "",
              "Comma-separated list of one or more URDF files that contain "
              "static links for the sensor configuration(s).");
DEFINE_bool(use_bag_transforms, true, "Whether to read, use and republish transforms from bags.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true, "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_string(save_state_filename, "",
              "Explicit name of the file to which the serialized state will be "
              "written before shutdown. If left empty, the filename will be "
              "inferred from the first bagfile's name as: "
              "<bag_filenames[0]>.pbstream");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");
DEFINE_double(skip_seconds, 0,
              "Optional amount of seconds to skip from the beginning "
              "(i.e. when the earliest bag starts.). ");

namespace cartographer_ros
{

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;
// We publish tf messages one second earlier than other messages. Under
// the assumption of higher frequency tf this should ensure that tf can
// always interpolate.
const rclcpp::Duration kDelay(1.0, 0);

using MapBuilderFactory = std::function<std::unique_ptr<::cartographer::mapping::MapBuilderInterface>(
    const ::cartographer::mapping::proto::MapBuilderOptions&)>;

std::vector<std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
    ComputeDefaultSensorIdsForMultipleBags(
        const std::vector<TrajectoryOptions>& bags_options,
        const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids)
{
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    std::vector<std::set<SensorId>> bags_sensor_ids;
    for (size_t i = 0; i < bags_options.size(); ++i)
    {
        std::string prefix;
        if (bags_options.size() > 1)
        {
            prefix = "bag_" + std::to_string(i + 1) + "_";
        }
        std::set<SensorId> unique_sensor_ids;
        for (const auto& sensor_id : expected_sensor_ids)
        {
            unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
        }
        bags_sensor_ids.push_back(unique_sensor_ids);
    }
    return bags_sensor_ids;
}

void RunOfflineNode(const MapBuilderFactory& map_builder_factory, rclcpp::Node::SharedPtr cartographer_offline_node)
{
    CHECK(!FLAGS_configuration_directory.empty()) << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basenames.empty()) << "-configuration_basenames is missing.";
    CHECK(!(FLAGS_bag_filenames.empty() && FLAGS_load_state_filename.empty()))
        << "-bag_filenames and -load_state_filename cannot both be unspecified.";

    // const std::vector<std::string> bag_filenames = absl::StrSplit(FLAGS_bag_filenames, ',', absl::SkipEmpty());
    std::regex regex(",");
    std::vector<std::string> bag_filenames;
    if (!FLAGS_bag_filenames.empty())
    {
        std::regex regex(",");
        std::vector<std::string> if_bag_filenames(
            std::sregex_token_iterator(FLAGS_bag_filenames.begin(), FLAGS_bag_filenames.end(), regex, -1),
            std::sregex_token_iterator());
        bag_filenames = if_bag_filenames;
    }
    cartographer_ros::NodeOptions node_options;
    // const std::vector<std::string> configuration_basenames =
    //     absl::StrSplit(FLAGS_configuration_basenames, ',', absl::SkipEmpty());
    std::vector<std::string> configuration_basenames(std::sregex_token_iterator(FLAGS_configuration_basenames.begin(),
                                                                                FLAGS_configuration_basenames.end(),
                                                                                regex, -1),
                                                     std::sregex_token_iterator());
    std::vector<TrajectoryOptions> bag_trajectory_options(1);
    std::tie(node_options, bag_trajectory_options.at(0)) =
        LoadOptions(FLAGS_configuration_directory, configuration_basenames.at(0));

    for (size_t bag_index = 1; bag_index < bag_filenames.size(); ++bag_index)
    {
        TrajectoryOptions current_trajectory_options;
        if (bag_index < configuration_basenames.size())
        {
            std::tie(std::ignore, current_trajectory_options) =
                LoadOptions(FLAGS_configuration_directory, configuration_basenames.at(bag_index));
        }
        else
        {
            current_trajectory_options = bag_trajectory_options.at(0);
        }
        bag_trajectory_options.push_back(current_trajectory_options);
    }
    if (bag_filenames.size() > 0)
    {
        CHECK_EQ(bag_trajectory_options.size(), bag_filenames.size());
    }

    // Since we preload the transform buffer, we should never have to wait for a
    // transform. When we finish processing the bag, we will simply drop any
    // remaining sensor data that cannot be transformed due to missing transforms.
    node_options.lookup_transform_timeout_sec = 0.;

    auto map_builder = map_builder_factory(node_options.map_builder_options);

    const std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();

    // auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(
        cartographer_offline_node->get_clock(), tf2::durationFromSec(10), cartographer_offline_node);

    std::vector<geometry_msgs::msg::TransformStamped> urdf_transforms;
    // const std::vector<std::string> urdf_filenames = absl::StrSplit(FLAGS_urdf_filenames, ',', absl::SkipEmpty());
    // for (const auto& urdf_filename : urdf_filenames)
    // {
    //     const auto current_urdf_transforms = ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer.get());
    //     urdf_transforms.insert(urdf_transforms.end(), current_urdf_transforms.begin(),
    //     current_urdf_transforms.end());
    // }
    if (!FLAGS_urdf_filenames.empty())
    {
        std::vector<std::string> urdf_filenames(
            std::sregex_token_iterator(FLAGS_urdf_filenames.begin(), FLAGS_urdf_filenames.end(), regex, -1),
            std::sregex_token_iterator());
        for (const auto& urdf_filename : urdf_filenames)
        {
            const auto current_urdf_transforms = ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer);
            urdf_transforms.insert(urdf_transforms.end(), current_urdf_transforms.begin(),
                                   current_urdf_transforms.end());
        }
    }

    tf_buffer->setUsingDedicatedThread(true);

    cartographer_ros::Cartographer node(node_options, bag_trajectory_options.at(0), FLAGS_collect_metrics);
    if (!FLAGS_load_state_filename.empty())
    {
        node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }

    // ::ros::Publisher tf_publisher = node.node_handle()->advertise<tf2_msgs::TFMessage>(kTfTopic, 1);
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher =
        cartographer_offline_node->create_publisher<tf2_msgs::msg::TFMessage>(kTfTopic, kLatestOnlyPublisherQueueSize);

    ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(cartographer_offline_node);

    // ::ros::Publisher clock_publisher = node.node_handle()->advertise<rosgraph_msgs::Clock>(kClockTopic, 1);
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher =
        cartographer_offline_node->create_publisher<rosgraph_msgs::msg::Clock>(kClockTopic,
                                                                               kLatestOnlyPublisherQueueSize);

    if (urdf_transforms.size() > 0)
    {
        static_tf_broadcaster.sendTransform(urdf_transforms);
    }

    // ros::AsyncSpinner async_spinner(kSingleThreaded);
    // async_spinner.start();
    rosgraph_msgs::msg::Clock clock;
    // auto clock_republish_timer = node.node_handle()->createWallTimer(
    //     ::ros::WallDuration(kClockPublishFrequencySec),
    //     [&clock_publisher, &clock](const ::ros::WallTimerEvent&) { clock_publisher.publish(clock); },
    //     false /* oneshot */, false /* autostart */);
    // auto clock_republish_timer = cartographer_offline_node->create_wall_timer(
    //     std::chrono::milliseconds(int(kClockPublishFrequencySec)),
    //     [&clock_publisher, &clock]() {
    //         clock_publisher->publish(clock);
    //     });

    std::vector<std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>> bag_expected_sensor_ids;

    if (configuration_basenames.size() == 1)
    {
        const auto node_options_expected_sensor_ids = ComputeExpectedSensorIds(bag_trajectory_options.front());
        const auto current_bag_expected_sensor_ids =
            ComputeDefaultSensorIdsForMultipleBags({bag_trajectory_options.front()}, node_options_expected_sensor_ids);
        bag_expected_sensor_ids = {bag_filenames.size(), current_bag_expected_sensor_ids.front()};
    }
    else
    {
        const auto node_options_expected_sensor_ids = ComputeExpectedSensorIds(bag_trajectory_options.front());
        bag_expected_sensor_ids =
            ComputeDefaultSensorIdsForMultipleBags(bag_trajectory_options, node_options_expected_sensor_ids);
    }
    CHECK_EQ(bag_expected_sensor_ids.size(), bag_filenames.size());

    std::map<std::pair<int /* bag_index */, std::string>, cartographer::mapping::TrajectoryBuilderInterface::SensorId>
        bag_topic_to_sensor_id;
    PlayableBagMultiplexer playable_bag_multiplexer(cartographer_offline_node);
    for (size_t current_bag_index = 0; current_bag_index < bag_filenames.size(); ++current_bag_index)
    {
        const std::string& bag_filename = bag_filenames.at(current_bag_index);
        if (!rclcpp::ok())
        {
            return;
        }
        for (const auto& expected_sensor_id : bag_expected_sensor_ids.at(current_bag_index))
        {
            const auto bag_resolved_topic =
                std::make_pair(static_cast<int>(current_bag_index), "/" + expected_sensor_id.id);
            if (bag_topic_to_sensor_id.count(bag_resolved_topic) != 0)
            {
                LOG(ERROR) << "Sensor " << expected_sensor_id.id << " of bag " << current_bag_index
                           << " resolves to topic " << bag_resolved_topic.second << " which is already used by "
                           << " sensor " << bag_topic_to_sensor_id.at(bag_resolved_topic).id;
            }
            bag_topic_to_sensor_id[bag_resolved_topic] = expected_sensor_id;
        }

        auto serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
        playable_bag_multiplexer.AddPlayableBag(
            PlayableBag(bag_filename, current_bag_index, kDelay,
                        // PlayableBag::FilteringEarlyMessageHandler is used to get an early
                        // peek at the tf messages in the bag and insert them into 'tf_buffer'.
                        // When a message is retrieved by GetNextMessage() further below,
                        // we will have already inserted further 'kDelay' seconds worth of
                        // transforms into 'tf_buffer' via this lambda.
                        [&tf_publisher, &tf_buffer, cartographer_offline_node,
                         serializer](std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg)
                        {
                            // Can't filter by type as in ROS1
                            if (msg->topic_name == kTfTopic || msg->topic_name == kTfStaticTopic)
                            {
                                if (FLAGS_use_bag_transforms)
                                {
                                    // const auto tf_message = msg.instantiate<tf2_msgs::TFMessage>();
                                    // tf_publisher.publish(tf_message);
                                    tf2_msgs::msg::TFMessage tf_message;
                                    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                                    try
                                    {
                                        serializer.deserialize_message(&serialized_msg, &tf_message);
                                        for (auto& transform : tf_message.transforms)
                                        {
                                            try
                                            {
                                                // We need to keep 'tf_buffer' small because it becomes very
                                                // inefficient otherwise. We make sure that tf_messages are
                                                // published before any data messages, so that tf lookups
                                                // always work.
                                                tf_buffer->setTransform(transform, "unused_authority",
                                                                        msg->topic_name == kTfStaticTopic);
                                            }
                                            catch (const tf2::TransformException& ex)
                                            {
                                                LOG(WARNING) << ex.what();
                                            }
                                        }
                                        tf_publisher->publish(tf_message);
                                    }
                                    catch (const rclcpp::exceptions::RCLError& rcl_error)
                                    {
                                        return true;
                                    }
                                }
                                // Tell 'PlayableBag' to filter the tf message since there is no
                                // further use for it.
                                return false;
                            }
                            else
                            {
                                return true;
                            }
                        }));
    }

    std::set<std::string> bag_topics;
    std::stringstream bag_topics_string;
    for (const auto& topic : playable_bag_multiplexer.topics())
    {
        std::string resolved_topic =
            cartographer_offline_node->get_node_base_interface()->resolve_topic_or_service_name(topic, false);
        bag_topics.insert(resolved_topic);
        bag_topics_string << resolved_topic << ",";
    }
    bool print_topics = false;
    for (const auto& entry : bag_topic_to_sensor_id)
    {
        const std::string& resolved_topic = entry.first.second;
        if (bag_topics.count(resolved_topic) == 0)
        {
            LOG(WARNING) << "Expected resolved topic \"" << resolved_topic << "\" not found in bag file(s).";
            print_topics = true;
        }
    }
    if (print_topics)
    {
        LOG(WARNING) << "Available topics in bag file(s) are " << bag_topics_string.str();
    }

    std::unordered_map<int, int> bag_index_to_trajectory_id;
    const rclcpp::Time begin_time =
        // If no bags were loaded, we cannot peek the time of first message.
        playable_bag_multiplexer.IsMessageAvailable() ? playable_bag_multiplexer.PeekMessageTime() : rclcpp::Clock(RCL_ROS_TIME).now();

    auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
    auto multi_echo_laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
    auto pcl2_serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
    auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
    auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
    auto nav_sat_fix_serializer = rclcpp::Serialization<sensor_msgs::msg::NavSatFix>();
    auto landmark_list_serializer = rclcpp::Serialization<cartographer_ros_msgs::msg::LandmarkList>();

    while (playable_bag_multiplexer.IsMessageAvailable())
    {
        if (!rclcpp::ok())
        {
            return;
        }

        const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
        const rosbag2_storage::SerializedBagMessage& msg = std::get<0>(next_msg_tuple);
        const int bag_index = std::get<1>(next_msg_tuple);
        const std::string topic_type = std::get<2>(next_msg_tuple);
        const bool is_last_message_in_bag = std::get<3>(next_msg_tuple);

        if (msg.time_stamp < (begin_time.nanoseconds() + rclcpp::Duration(FLAGS_skip_seconds, 0).nanoseconds()))
        {
            continue;
        }

        int trajectory_id;
        // Lazily add trajectories only when the first message arrives in order
        // to avoid blocking the sensor queue.
        if (bag_index_to_trajectory_id.count(bag_index) == 0)
        {
            trajectory_id =
                node.AddOfflineTrajectory(bag_expected_sensor_ids.at(bag_index), bag_trajectory_options.at(bag_index));
            CHECK(bag_index_to_trajectory_id
                      .emplace(std::piecewise_construct, std::forward_as_tuple(bag_index),
                               std::forward_as_tuple(trajectory_id))
                      .second);
            LOG(INFO) << "Assigned trajectory " << trajectory_id << " to bag " << bag_filenames.at(bag_index);
        }
        else
        {
            trajectory_id = bag_index_to_trajectory_id.at(bag_index);
        }

        const auto bag_topic = std::make_pair(
            bag_index,
            cartographer_offline_node->get_node_base_interface()->resolve_topic_or_service_name(msg.topic_name, false));
        auto it = bag_topic_to_sensor_id.find(bag_topic);

        if (it != bag_topic_to_sensor_id.end())
        {
            const std::string& sensor_id = it->second.id;
            if (topic_type == "sensor_msgs/msg/LaserScan")
            {
                rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
                sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
                laser_scan_serializer.deserialize_message(&serialized_msg, laser_scan_msg.get());
                node.HandleLaserScanMessage(trajectory_id, sensor_id, laser_scan_msg);
            }
            else if (topic_type == "sensor_msgs/msg/MultiEchoLaserScan")
            {
                rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
                sensor_msgs::msg::MultiEchoLaserScan::SharedPtr multi_echo_laser_scan_msg =
                    std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
                multi_echo_laser_scan_serializer.deserialize_message(&serialized_msg, multi_echo_laser_scan_msg.get());
                node.HandleMultiEchoLaserScanMessage(trajectory_id, sensor_id, multi_echo_laser_scan_msg);
            }
            else if (topic_type == "sensor_msgs/msg/PointCloud2")
            {
                rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
                sensor_msgs::msg::PointCloud2::SharedPtr pcl2_scan_msg =
                    std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl2_serializer.deserialize_message(&serialized_msg, pcl2_scan_msg.get());
                node.HandlePointCloud2Message(trajectory_id, sensor_id, pcl2_scan_msg);
            }
            else if (topic_type == "nav_msgs/msg/Odometry")
            {
                rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
                nav_msgs::msg::Odometry::SharedPtr odom_scan_msg = std::make_shared<nav_msgs::msg::Odometry>();
                odom_serializer.deserialize_message(&serialized_msg, odom_scan_msg.get());
                node.HandleOdometryMessage(trajectory_id, sensor_id, odom_scan_msg);
            }
            else if (topic_type == "sensor_msgs/msg/NavSatFix")
            {
                rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
                sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_msg =
                    std::make_shared<sensor_msgs::msg::NavSatFix>();
                nav_sat_fix_serializer.deserialize_message(&serialized_msg, nav_sat_fix_msg.get());
                node.HandleNavSatFixMessage(trajectory_id, sensor_id, nav_sat_fix_msg);
            }
            else if (topic_type == "cartographer_ros_msgs/msg/LandmarkList")
            {
                rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
                cartographer_ros_msgs::msg::LandmarkList::SharedPtr landmark_list_msg =
                    std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
                landmark_list_serializer.deserialize_message(&serialized_msg, landmark_list_msg.get());
                node.HandleLandmarkMessage(trajectory_id, sensor_id, landmark_list_msg);
            }
        }
        clock.clock = rclcpp::Time(msg.time_stamp);
        clock_publisher->publish(clock);
        rclcpp::spin_some(cartographer_offline_node);

        if (is_last_message_in_bag)
        {
            node.FinishTrajectory(trajectory_id);
        }
    }

    // Ensure the clock is republished after the bag has been finished, during the
    // final optimization, serialization, and optional indefinite spinning at the
    // end.
    // TODO from port: need a spin for the timer to tick
    auto clock_republish_timer =
        cartographer_offline_node->create_wall_timer(std::chrono::milliseconds(int(kClockPublishFrequencySec)),
                                                     [&clock_publisher, &clock]() { clock_publisher->publish(clock); });
    node.RunFinalOptimization();

    const std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
    const double wall_clock_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();

    LOG(INFO) << "Elapsed wall clock time: " << wall_clock_seconds << " s";
#ifdef __linux__
    timespec cpu_timespec = {};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec);
    LOG(INFO) << "Elapsed CPU time: " << (cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec) << " s";
    rusage usage;
    CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0) << strerror(errno);
    LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
#endif

    // Serialize unless we have neither a bagfile nor an explicit state filename.
    if (rclcpp::ok() && !(bag_filenames.empty() && FLAGS_save_state_filename.empty()))
    {
        const std::string state_output_filename = FLAGS_save_state_filename.empty()
                                                      //   ? absl::StrCat(bag_filenames.front(), ".pbstream")
                                                      ? bag_filenames.front() + ".pbstream"
                                                      : FLAGS_save_state_filename;
        LOG(INFO) << "Writing state to '" << state_output_filename << "'...";
        node.SerializeState(state_output_filename, true /* include_unfinished_submaps */);
    }
    if (FLAGS_keep_running)
    {
        LOG(INFO) << "Finished processing and waiting for shutdown.";
        rclcpp::spin(cartographer_offline_node);
    }
}

}  // namespace cartographer_ros

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    google::AllowCommandLineReparsing();
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    cartographer_ros::ScopedRosLogSink ros_log_sink;

    const cartographer_ros::MapBuilderFactory map_builder_factory =
        [](const ::cartographer::mapping::proto::MapBuilderOptions& map_builder_options)
    { return absl::make_unique<::cartographer::mapping::MapBuilder>(map_builder_options); };

    rclcpp::Node::SharedPtr cartographer_offline_node = rclcpp::Node::make_shared("cartographer_offline_node");
    cartographer_ros::RunOfflineNode(map_builder_factory, cartographer_offline_node);

    rclcpp::shutdown();
}

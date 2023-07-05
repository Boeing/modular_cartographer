// #include <boost/program_options.hpp>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer_ros/node.h>
#include <cartographer_ros/node_options.h>
#include <cartographer_ros/ros_log_sink.h>
#include <cartographer_ros/urdf_reader.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <urdf/model.h>

#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"

// namespace po = boost::program_options;

namespace cartographer_ros
{

void testFindConstraint(const std::string& configuration_directory, const std::string& urdf_filename,
                        const std::string& pbstream_filename, const std::string& rosbag_filename)
{
    LOG(INFO) << "TestFindConstraint";

    cartographer_ros::NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) = LoadOptions(configuration_directory, "cartographer.lua");

    LOG(INFO) << "Loading URDF";

    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);

    tf_buffer->setUsingDedicatedThread(true);
    ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer);

    LOG(INFO) << "Building MapBuilder";
    auto map_builder_bridge = std::make_unique<MapBuilderBridge>(node_options, tf_buffer);
    std::ifstream file(pbstream_filename);
    map_builder_bridge->LoadState(file, true);

    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    //    SensorId odom{SensorType::ODOMETRY, "odom"};

    //
    // Two lidars
    //
    SensorId front_laser{SensorType::RANGE, "/front_laser/scan"};
    SensorId back_laser{SensorType::RANGE, "/rear_laser/scan"};
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> expected_sensor_ids = {front_laser,
                                                                                                       back_laser};
    //
    // One lidar
    //
    // SensorId laser{SensorType::RANGE, "/scan_multi"};
    // const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> expected_sensor_ids = {laser};

    const int trajectory_id = map_builder_bridge->AddTrajectory(expected_sensor_ids, trajectory_options);

    rosbag2_cpp::Reader bag_reader;
    bag_reader.open(rosbag_filename);

    std::vector<rosbag2_storage::TopicMetadata> topics = bag_reader.get_all_topics_and_types();
    for (const rosbag2_storage::TopicMetadata& topic_info : topics)
    {
        LOG(INFO) << "Topic: " << topic_info.name;
    }

    LOG(INFO) << "Loading data...";

    auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();

    //
    // Two lidars
    //
    bool front_laser_read = false;
    bool back_laser_read = false;
    sensor_msgs::msg::LaserScan::SharedPtr front_laser_msgs = std::make_shared<sensor_msgs::msg::LaserScan>();
    sensor_msgs::msg::LaserScan::SharedPtr back_laser_msgs = std::make_shared<sensor_msgs::msg::LaserScan>();
    //
    // One lidar
    //
    // bool laser_read = false;
    // sensor_msgs::msg::LaserScan::SharedPtr laser_msgs = std::make_shared<sensor_msgs::msg::LaserScan>();

    while (bag_reader.has_next())
    {
        auto message = bag_reader.read_next();
        //
        // Two lidars
        //
        if (message->topic_name == front_laser.id && !front_laser_read)
        {
            front_laser_read = true;
            rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
            laser_scan_serializer.deserialize_message(&serialized_msg, front_laser_msgs.get());
        }
        else if (message->topic_name == back_laser.id && !back_laser_read)
        {
            back_laser_read = false;
            rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
            laser_scan_serializer.deserialize_message(&serialized_msg, back_laser_msgs.get());
        }
        if (front_laser_read && back_laser_read)
        {
            break;
        }
        //
        // One lidar
        //
        // if (message->topic_name == laser.id && !laser_read)
        // {
        //     laser_read = true;
        //     rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
        //     laser_scan_serializer.deserialize_message(&serialized_msg, laser_msgs.get());
        // }
        // if (laser_read)
        // {
        //     break;
        // }
    }

    //
    // Two lidars
    //
    map_builder_bridge->sensor_bridge(trajectory_id)->HandleLaserScanMessage(front_laser.id, front_laser_msgs);
    map_builder_bridge->sensor_bridge(trajectory_id)->HandleLaserScanMessage(back_laser.id, back_laser_msgs);
    //
    // One lidar
    //
    // map_builder_bridge->sensor_bridge(trajectory_id)->HandleLaserScanMessage(laser.id, laser_msgs);

    LOG(INFO) << "Running final optimization";

    map_builder_bridge->RunFinalOptimization();
}

}  // namespace cartographer_ros

DEFINE_string(configuration_directory, "", "Cartographer configuration directory");
DEFINE_string(pbstream_filename, "map.pbstream", "Pbstream destination");
DEFINE_string(urdf_filename, "", "URDF");
// DEFINE_bool(merged_lasers, true, "Used merged lasers or not");
DEFINE_string(rosbag_filename, "", "Rosbag");

int main(int argc, char** argv)
{
    // google::InitGoogleLogging(argv[0]);
    rclcpp::init(argc, argv);

    google::AllowCommandLineReparsing();
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    // std::string configuration_directory;
    // std::string pbstream_filename;
    // std::string urdf_filename;
    // std::string rosbag_filename;

    // po::options_description desc("Options");
    // try
    // {
    //     // clang-format off
    //     desc.add_options()
    //     ("help,h", "Help Screen")
    //     ("configuration_directory", po::value<std::string>(&configuration_directory)->required(), "Cartographer
    //     configuration directory")
    //     ("pbstream", po::value<std::string>(&pbstream_filename)->required(), "Pbstream map")
    //     ("urdf", po::value<std::string>(&urdf_filename)->required(), "URDF")
    //     ("rosbag", po::value<std::string>(&rosbag_filename)->required(), "Rosbag");
    //     // clang-format on

    //     po::variables_map vm;
    //     po::store(po::parse_command_line(argc, argv, desc), vm);

    //     if (vm.count("help"))
    //     {
    //         std::cout << "display_pattern: " << std::endl << desc << std::endl;
    //         return EXIT_SUCCESS;
    //     }

    //     po::notify(vm);
    // }
    // catch (const po::error& e)
    // {
    //     std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    //     std::cerr << desc << std::endl;
    //     return EXIT_FAILURE;
    // }

    cartographer_ros::ScopedRosLogSink ros_log_sink;

    std::string temp_configuration_directory;
    std::string temp_urdf_filename;
    std::string temp_pbstream_filename;
    // bool temp_merged_lasers;
    std::string temp_rosbag_filename;

    // https://stackoverflow.com/questions/54295618/how-to-know-if-a-gflag-was-provided-in-the-command-line
    if (!gflags::GetCommandLineFlagInfoOrDie("configuration_directory").is_default)
    {
        temp_configuration_directory = FLAGS_configuration_directory;
    }
    else
    {
        throw std::runtime_error("Missing required param: configuration_directory");
    }
    if (!gflags::GetCommandLineFlagInfoOrDie("pbstream_filename").is_default)
    {
        temp_pbstream_filename = FLAGS_pbstream_filename;
    }
    else
    {
        throw std::runtime_error("Missing required param: pbstream_filename");
    }
    if (!gflags::GetCommandLineFlagInfoOrDie("urdf_filename").is_default)
    {
        temp_urdf_filename = FLAGS_urdf_filename;
    }
    else
    {
        throw std::runtime_error("Missing required param: urdf_filename");
    }
    // if (!gflags::GetCommandLineFlagInfoOrDie("merged_lasers").is_default)
    // {
    //     temp_merged_lasers = FLAGS_merged_lasers;
    // }
    // else
    // {
    //     temp_merged_lasers = false;
    // }
    if (!gflags::GetCommandLineFlagInfoOrDie("rosbag_filename").is_default)
    {
        temp_rosbag_filename = FLAGS_rosbag_filename;
    }
    else
    {
        throw std::runtime_error("Missing required param: rosbag_filename");
    }

    const std::string configuration_directory = temp_configuration_directory;
    const std::string urdf_filename = temp_urdf_filename;
    // const bool merged_lasers = temp_merged_lasers;
    const std::string pbstream_filename = temp_pbstream_filename;
    const std::string rosbag_filename = temp_rosbag_filename;

    LOG(INFO) << "Configuration directory: " << configuration_directory;
    LOG(INFO) << "URDF file: " << urdf_filename;
    // LOG(INFO) << "Use merged_lasers: " << merged_lasers;
    LOG(INFO) << "Pbstream file: " << pbstream_filename;
    LOG(INFO) << "Rosbag file: " << rosbag_filename;

    cartographer_ros::testFindConstraint(configuration_directory, urdf_filename, pbstream_filename, rosbag_filename);

    rclcpp::shutdown();
}

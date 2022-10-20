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
#include "gflags/gflags.h"

#include <memory>
#include <string>
#include <vector>

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
    SensorId front_laser{SensorType::RANGE, "/sick_s300_front/scan"};
    SensorId back_laser{SensorType::RANGE, "/sick_s300_back/scan"};
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> expected_sensor_ids = {front_laser,
                                                                                                       back_laser};

    const int trajectory_id = map_builder_bridge->AddTrajectory(expected_sensor_ids, trajectory_options);

    rosbag2_cpp::Reader bag_reader;
    bag_reader.open(rosbag_filename);

    std::vector<rosbag2_storage::TopicMetadata> topics = bag_reader.get_all_topics_and_types();
    for (const rosbag2_storage::TopicMetadata topic_info : topics) 
    {
        LOG(INFO) << "Topic: " << topic_info.name;
    }

    LOG(INFO) << "Loading data...";

    auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();

    bool front_laser_read = false;
    bool back_laser_read = false;
    sensor_msgs::msg::LaserScan::SharedPtr front_laser_msgs = std::make_shared<sensor_msgs::msg::LaserScan>();
    sensor_msgs::msg::LaserScan::SharedPtr back_laser_msgs = std::make_shared<sensor_msgs::msg::LaserScan>();

    while (bag_reader.has_next()) 
    {
        auto message = bag_reader.read_next();
        if (message->topic_name == front_laser.id && !front_laser_read)
        {
            front_laser_read = true;
            rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
            laser_scan_serializer.deserialize_message(&serialized_msg, front_laser_msgs.get());
        } else if (message->topic_name == back_laser.id && !back_laser_read)
        {
            back_laser_read = false;
            rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
            laser_scan_serializer.deserialize_message(&serialized_msg, back_laser_msgs.get());            
        }
        if (front_laser_read && back_laser_read)
        {
            break;
        }

    }

    map_builder_bridge->sensor_bridge(trajectory_id)
        ->HandleLaserScanMessage(front_laser.id, front_laser_msgs);
    map_builder_bridge->sensor_bridge(trajectory_id)
        ->HandleLaserScanMessage(back_laser.id, back_laser_msgs);

    LOG(INFO) << "Running final optimization";

    map_builder_bridge->RunFinalOptimization();
}

}  // namespace cartographer_ros

DEFINE_string(configuration_directory, "", "Cartographer configuration directory");
DEFINE_string(pbstream_filename, "map.pbstream", "Pbstream destination");
DEFINE_string(urdf_filename, "", "URDF");
DEFINE_string(rosbag_filename, "", "Rosbag");

int main(int argc, char** argv)
{
    // google::InitGoogleLogging(argv[0]);
    rclcpp::init(argc, argv);

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
    //     ("configuration_directory", po::value<std::string>(&configuration_directory)->required(), "Cartographer configuration directory")
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
    cartographer_ros::testFindConstraint(FLAGS_configuration_directory, FLAGS_urdf_filename, FLAGS_pbstream_filename, FLAGS_rosbag_filename);

    rclcpp::shutdown();
}

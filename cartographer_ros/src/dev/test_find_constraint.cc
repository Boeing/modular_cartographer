#include <boost/program_options.hpp>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer_ros/node.h>
#include <cartographer_ros/node_options.h>
#include <cartographer_ros/ros_log_sink.h>
#include <cartographer_ros/urdf_reader.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <urdf/model.h>

#include <memory>
#include <string>
#include <vector>

namespace po = boost::program_options;

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
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_buffer->setUsingDedicatedThread(true);
    ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer.get());

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

    rosbag::Bag bag(rosbag_filename, rosbag::bagmode::Read);

    rosbag::View view(bag);
    for (const auto info : view.getConnections())
        LOG(INFO) << "Topic: " << info->topic;

    rosbag::View front_laser_view(bag, rosbag::TopicQuery(front_laser.id));
    rosbag::View back_laser_view(bag, rosbag::TopicQuery(back_laser.id));
    //    rosbag::View odom_view(bag, rosbag::TopicQuery(odom.id));

    LOG(INFO) << "Loading data...";

    auto front_laser_msgs = front_laser_view.begin();
    auto back_laser_msgs = back_laser_view.begin();
    //    auto odom_msgs = odom_view.begin();

    if (front_laser_msgs == front_laser_view.end())
        ROS_FATAL("No front laser message");

    if (back_laser_msgs == back_laser_view.end())
        ROS_FATAL("No back laser message");

    map_builder_bridge->sensor_bridge(trajectory_id)
        ->HandleLaserScanMessage(front_laser.id, front_laser_msgs->instantiate<sensor_msgs::LaserScan>());
    map_builder_bridge->sensor_bridge(trajectory_id)
        ->HandleLaserScanMessage(back_laser.id, back_laser_msgs->instantiate<sensor_msgs::LaserScan>());

    LOG(INFO) << "Running final optimization";

    map_builder_bridge->RunFinalOptimization();
}

}  // namespace cartographer_ros

int main(int argc, char** argv)
{
    // google::InitGoogleLogging(argv[0]);

    ::ros::init(argc, argv, "test_find_constraint_node");
    ::ros::start();

    std::string configuration_directory;
    std::string pbstream_filename;
    std::string urdf_filename;
    std::string rosbag_filename;

    po::options_description desc("Options");
    try
    {
        // clang-format off
        desc.add_options()
        ("help,h", "Help Screen")
        ("configuration_directory", po::value<std::string>(&configuration_directory)->required(), "Cartographer configuration directory")
        ("pbstream", po::value<std::string>(&pbstream_filename)->required(), "Pbstream map")
        ("urdf", po::value<std::string>(&urdf_filename)->required(), "URDF")
        ("rosbag", po::value<std::string>(&rosbag_filename)->required(), "Rosbag");
        // clang-format on

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "display_pattern: " << std::endl << desc << std::endl;
            return EXIT_SUCCESS;
        }

        po::notify(vm);
    }
    catch (const po::error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_FAILURE;
    }

    cartographer_ros::ScopedRosLogSink ros_log_sink;
    cartographer_ros::testFindConstraint(configuration_directory, urdf_filename, pbstream_filename, rosbag_filename);

    ::ros::shutdown();
}

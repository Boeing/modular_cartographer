#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_bool(collect_metrics, false, "Collect metrics or not.");

namespace cartographer_ros
{
namespace
{

void Run()
{
    std::string temp_configuration_directory;
    bool temp_collect_metrics;

    // https://stackoverflow.com/questions/54295618/how-to-know-if-a-gflag-was-provided-in-the-command-line
    if (!gflags::GetCommandLineFlagInfoOrDie("configuration_directory").is_default)
    {
        temp_configuration_directory = FLAGS_configuration_directory;
    }
    else
    {
        throw std::runtime_error("Missing required param: configuration_directory");
    }
    if (!gflags::GetCommandLineFlagInfoOrDie("collect_metrics").is_default)
    {
        temp_collect_metrics = FLAGS_collect_metrics;
    }
    else
    {
        temp_collect_metrics = false;
    }

    const std::string configuration_directory = temp_configuration_directory;
    const bool collect_metrics = temp_collect_metrics;

    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) = LoadOptions(configuration_directory, "cartographer.lua");

    auto node = std::make_shared<cartographer_ros::Cartographer>(node_options, trajectory_options, collect_metrics);

    rclcpp::spin(node);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv)
{
    ::rclcpp::init(argc, argv);

    google::AllowCommandLineReparsing();
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty()) << "-configuration_directory is missing.";

    cartographer_ros::ScopedRosLogSink ros_log_sink;
    cartographer_ros::Run();
    ::rclcpp::shutdown();
}

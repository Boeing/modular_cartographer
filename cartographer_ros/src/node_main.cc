#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

template <typename T> T get_param_or_throw(const std::string& param_name)
{
    if (ros::param::has(param_name))
    {
        T param_val;
        if (ros::param::get(param_name, param_val))
        {
            return param_val;
        }
    }
    throw std::runtime_error("Missing required param: " + param_name);
}

template <typename T> T get_param_with_default_warn(const std::string& param_name, const T& default_val)
{
    if (ros::param::has(param_name))
    {
        T param_val;
        if (ros::param::get(param_name, param_val))
        {
            return param_val;
        }
    }
    ROS_WARN_STREAM("Using default value for " << param_name << "=" << default_val);
    return default_val;
}

namespace cartographer_ros
{
namespace
{



void Run()
{
    const std::string configuration_directory = get_param_or_throw<std::string>("~configuration_directory");
    const bool collect_metrics = get_param_with_default_warn<bool>("~collect_metrics", false);

    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);

    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) = LoadOptions(configuration_directory, "cartographer.lua");

    Node node(node_options, trajectory_options, &tf_buffer, collect_metrics);

    ::ros::spin();

    node.FinishAllTrajectories();
    node.RunFinalOptimization();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ::ros::init(argc, argv, "cartographer_node");
    ::ros::start();

    cartographer_ros::ScopedRosLogSink ros_log_sink;
    cartographer_ros::Run();
    ::ros::shutdown();
}

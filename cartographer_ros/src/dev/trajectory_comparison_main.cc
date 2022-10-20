/*
 * Copyright 2018 The Cartographer Authors
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

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/msg/tf_message.hpp"

DEFINE_string(bag_filename, "",
              "Bag file containing TF messages of the trajectory that will be "
              "compared against the trajectory in the .pbstream file.");
DEFINE_string(tf_parent_frame, "map", "The parent frame ID of the TF trajectory from the bag file.");
DEFINE_string(tf_child_frame, "base_link", "The child frame ID of the TF trajectory from the bag file.");
DEFINE_string(tf_topic, "tf", "Topic where TF messages are being published.");
DEFINE_string(pbstream_filename, "",
              "Proto stream file containing the pose graph. The last "
              "trajectory will be used for comparison.");

namespace cartographer_ros
{
namespace
{

double FractionSmallerThan(const std::vector<double>& v, double x)
{
    return static_cast<double>(std::count_if(v.begin(), v.end(), [=](double value) { return value < x; })) / v.size();
}

std::string QuantilesToString(std::vector<double>* v)
{
    if (v->empty())
        return "(empty vector)";
    std::sort(v->begin(), v->end());
    std::stringstream result;
    const int kNumQuantiles = 10;
    for (int i = 0; i < kNumQuantiles; ++i)
    {
        auto value = v->at(v->size() * i / kNumQuantiles);
        auto percentage = 100 * i / kNumQuantiles;
        result << percentage << "%: " << value << "\n";
    }
    result << "100%: " << v->back() << "\n";
    return result.str();
}

void Run(const std::string& pbstream_filename, const std::string& bag_filename)
{
    cartographer::mapping::proto::PoseGraph pose_graph_proto =
        cartographer::io::DeserializePoseGraphFromFile(pbstream_filename);
    const cartographer::mapping::proto::Trajectory& last_trajectory_proto =
        *pose_graph_proto.mutable_trajectory()->rbegin();
    const cartographer::transform::TransformInterpolationBuffer transform_interpolation_buffer(last_trajectory_proto);

    // Declare serializer for TFMessages
    auto tf_serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();

    rosbag2_cpp::Reader bag_reader;
    bag_reader.open(bag_filename);

    std::vector<double> deviation_translation, deviation_rotation;
    const double signal_maximum = std::numeric_limits<double>::max();
    while (bag_reader.has_next())
    {
        auto message = bag_reader.read_next();
        if (message->topic_name != FLAGS_tf_topic)
        {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
        tf2_msgs::msg::TFMessage::SharedPtr msg = std::make_shared<tf2_msgs::msg::TFMessage>();
        ;
        tf_serializer.deserialize_message(&serialized_msg, msg.get());

        for (auto& transform : msg->transforms)
        {
            if (transform.header.frame_id != FLAGS_tf_parent_frame || transform.child_frame_id != FLAGS_tf_child_frame)
            {
                continue;
            }
            const cartographer::common::Time transform_time = FromRos(transform.header.stamp);
            if (!transform_interpolation_buffer.Has(transform_time))
            {
                deviation_translation.push_back(signal_maximum);
                deviation_rotation.push_back(signal_maximum);
                continue;
            }
            auto optimized_transform = transform_interpolation_buffer.Lookup(transform_time);
            auto published_transform = ToRigid3d(transform);
            deviation_translation.push_back(
                (published_transform.translation() - optimized_transform.translation()).norm());
            deviation_rotation.push_back(
                published_transform.rotation().angularDistance(optimized_transform.rotation()));
        }
    }
    LOG(INFO) << "Distribution of translation difference:\n" << QuantilesToString(&deviation_translation);
    LOG(INFO) << "Distribution of rotation difference:\n" << QuantilesToString(&deviation_rotation);
    LOG(INFO) << "Fraction of translation difference smaller than 1m: "
              << FractionSmallerThan(deviation_translation, 1);
    LOG(INFO) << "Fraction of translation difference smaller than 0.1m: "
              << FractionSmallerThan(deviation_translation, 0.1);
    LOG(INFO) << "Fraction of translation difference smaller than 0.05m: "
              << FractionSmallerThan(deviation_translation, 0.05);
    LOG(INFO) << "Fraction of translation difference smaller than 0.01m: "
              << FractionSmallerThan(deviation_translation, 0.01);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv)
{
    FLAGS_alsologtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::SetUsageMessage("\n\n"
                            "This compares a trajectory from a bag file against the "
                            "last trajectory in a pbstream file.\n");
    google::ParseCommandLineFlags(&argc, &argv, true);
    CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
    CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
    ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_bag_filename);
}

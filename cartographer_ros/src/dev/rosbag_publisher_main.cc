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

#include "cartographer/common/time.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/time_conversion.h"
#include "gflags/gflags.h"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <string>

DEFINE_string(bag_filename, "", "Bag to publish.");

namespace cartographer_ros
{

class BagPublisher : public rclcpp::Node
{
  public:
    BagPublisher(const std::string& bag_filename)
    : Node("rosbag_publisher"), bag_filename_(bag_filename)
    {

      auto pcl2_serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
      auto multi_echo_laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
      auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
      auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
      auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
      auto tf_serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
      rclcpp::Time aux_time;
      rcl_ret_t publish_result;

      bag_reader_.open(bag_filename);
      // TO-DO: Check use_sim_time
      topics_ = bag_reader_.get_all_topics_and_types();
      for (const rosbag2_storage::TopicMetadata topic_info : topics_) 
      {
        if (topic_info.type == "sensor_msgs/msg/PointCloud2")
        {
          topic_to_publisher_[topic_info.name] = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_info.name, kQueueSize_)->get_publisher_handle();
        } else if (topic_info.type == "sensor_msgs/msg/MultiEchoLaserScan")
        {
          topic_to_publisher_[topic_info.name] = this->create_publisher<sensor_msgs::msg::MultiEchoLaserScan>(topic_info.name, kQueueSize_)->get_publisher_handle();
        } else if (topic_info.type == "sensor_msgs/msg/LaserScan")
        {
          topic_to_publisher_[topic_info.name] = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_info.name, kQueueSize_)->get_publisher_handle();
        } else if (topic_info.type == "sensor_msgs/msg/Imu")
        {
          topic_to_publisher_[topic_info.name] = this->create_publisher<sensor_msgs::msg::Imu>(topic_info.name, kQueueSize_)->get_publisher_handle();
        } else if (topic_info.type == "nav_msgs/msg/Odometry")
        {
          topic_to_publisher_[topic_info.name] = this->create_publisher<nav_msgs::msg::Odometry>(topic_info.name, kQueueSize_)->get_publisher_handle();
        } else if (topic_info.type == "tf2_msgs/msg/TFMessage")
        {
          topic_to_publisher_[topic_info.name] = this->create_publisher<tf2_msgs::msg::TFMessage>(topic_info.name, kQueueSize_)->get_publisher_handle();
        }
      }
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      CHECK(rclcpp::ok());

      current_start_ = this->now();
      bag_start_ = rclcpp::Time(bag_reader_.get_metadata().starting_time.time_since_epoch().count());
      // Offset between bag start and current time
      bag_to_current_ = current_start_ - bag_start_;
      while (bag_reader_.has_next()) 
      {
        if (!rclcpp::ok())
        {
          break;
        }
        auto message = bag_reader_.read_next();
        // Time elapsed (offset) between bag start and current message time stamp
        rclcpp::Duration after_bag_start = rclcpp::Time(message->time_stamp) - bag_start_;
        // Time at which we need to publish the message start time + message offset
        rclcpp::Time planned_publish_time = current_start_ + after_bag_start;
        // To be replaced by sleep_until in humble
        rclcpp::sleep_for(std::chrono::nanoseconds((planned_publish_time - this->now()).nanoseconds()));        
        // Search in metadata the current topic
        for (const rosbag2_storage::TopicMetadata topic_info : topics_) 
        {
          // If topics names match, process message
          if(topic_info.name == message->topic_name)
          {
            if (topic_info.type == "sensor_msgs/msg/PointCloud2")
            {
              rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
              sensor_msgs::msg::PointCloud2::SharedPtr msg =
                  std::make_shared<sensor_msgs::msg::PointCloud2>();
              pcl2_serializer.deserialize_message(&serialized_msg, msg.get());
              aux_time = msg->header.stamp;
              aux_time += bag_to_current_;
              msg->header.stamp = aux_time;
              publish_result = rcl_publish(topic_to_publisher_.at(message->topic_name).get(), msg.get(), NULL);
            } else if (topic_info.type == "sensor_msgs/msg/MultiEchoLaserScan")
            {
              rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
              sensor_msgs::msg::MultiEchoLaserScan::SharedPtr msg =
                std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
              multi_echo_laser_scan_serializer.deserialize_message(&serialized_msg, msg.get());
              aux_time = msg->header.stamp;
              aux_time += bag_to_current_;
              msg->header.stamp = aux_time;
              publish_result = rcl_publish(topic_to_publisher_.at(message->topic_name).get(), msg.get(), NULL);
            } else if (topic_info.type == "sensor_msgs/msg/LaserScan")
            {
              rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
              sensor_msgs::msg::LaserScan::SharedPtr msg =
                std::make_shared<sensor_msgs::msg::LaserScan>();
              laser_scan_serializer.deserialize_message(&serialized_msg, msg.get());
              aux_time = msg->header.stamp;
              aux_time += bag_to_current_;
              msg->header.stamp = aux_time;
              publish_result = rcl_publish(topic_to_publisher_.at(message->topic_name).get(), msg.get(), NULL);
            } else if (topic_info.type == "sensor_msgs/msg/Imu")
            {
              rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
              sensor_msgs::msg::Imu::SharedPtr msg =
                std::make_shared<sensor_msgs::msg::Imu>();
              imu_serializer.deserialize_message(&serialized_msg, msg.get());
              aux_time = msg->header.stamp;
              aux_time += bag_to_current_;
              msg->header.stamp = aux_time;
              publish_result = rcl_publish(topic_to_publisher_.at(message->topic_name).get(), msg.get(), NULL);
            } else if (topic_info.type == "nav_msgs/msg/Odometry")
            {
              rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
              nav_msgs::msg::Odometry::SharedPtr msg =
                std::make_shared<nav_msgs::msg::Odometry>();
              odom_serializer.deserialize_message(&serialized_msg, msg.get());
              aux_time = msg->header.stamp;
              aux_time += bag_to_current_;
              msg->header.stamp = aux_time;
              publish_result = rcl_publish(topic_to_publisher_.at(message->topic_name).get(), msg.get(), NULL);
            } else if (topic_info.type == "tf2_msgs/msg/TFMessage")
            {
              rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
              tf2_msgs::msg::TFMessage::SharedPtr msg
                = std::make_shared<tf2_msgs::msg::TFMessage>();;
              tf_serializer.deserialize_message(&serialized_msg, msg.get());
              for (auto& transform : msg->transforms)
              {
                aux_time = transform.header.stamp;
                aux_time += bag_to_current_;
                transform.header.stamp = aux_time;
              }
              publish_result = rcl_publish(topic_to_publisher_.at(message->topic_name).get(), msg.get(), NULL);
            } else
            {
              LOG(WARNING) << "Skipping message with type " << topic_info.type;
              break;
            }
            if (publish_result != RCL_RET_OK)
            {
              LOG(WARNING) << "Error publishing message with type " << topic_info.type;
            }
            // Message processed, skip to next.
            break;
          }
        }
      }
    }

  private:
    std::string bag_filename_;
    rosbag2_cpp::Reader bag_reader_;
    std::vector<rosbag2_storage::TopicMetadata> topics_;
    // Publisher handlers stored as rcl_publisher_t
    // Publication will be done using rcl_publish()
    std::map<std::string, std::shared_ptr<rcl_publisher_t>> topic_to_publisher_;
    rclcpp::Time current_start_, bag_start_;
    rclcpp::Duration bag_to_current_ = rclcpp::Duration(1.0, 0);
    const int kQueueSize_ = 1;
};

} // end namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage("\n\n"
                          "This replays and publishes messages from a given bag file, modifying "
                          "their header timestamps to match current ROS time.\n\n"
                          "Messages are published in the same sequence and with the same delay "
                          "they were recorded."
                          "Contrary to rosbag play, it does not publish a clock, so time is"
                          "hopefully smoother and it should be possible to reproduce timing"
                          "issues.\n"
                          "It only plays message types related to Cartographer.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  auto node = std::make_shared<cartographer_ros::BagPublisher>(FLAGS_bag_filename);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H

#include <ctime>

#include "glog/logging.h"

namespace cartographer_ros
{

// Makes Google logging use ROS logging for output while an instance of this
// class exists.
class ScopedRosLogSink : public ::google::LogSink
{
  public:
    ScopedRosLogSink();
    ~ScopedRosLogSink() override;

    void send(::google::LogSeverity severity, const char* filename, const char* base_filename, int line,
              const struct std::tm* tm_time, const char* message, size_t message_len) override;

    void WaitTillSent() override;

  private:
    bool will_die_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H

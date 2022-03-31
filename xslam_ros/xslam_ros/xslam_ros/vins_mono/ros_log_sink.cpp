#include "xslam_ros/vins_mono/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "glog/log_severity.h"
#include "ros/console.h"

namespace xslam_ros {
namespace vins_mono {
namespace {

const char* GetBasename(const char* filepath) 
{
    const char* base = std::strrchr(filepath, '/');
    return base ? (base + 1) : filepath;
}

}  // namespace

ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }

void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
  switch (severity) {
    case ::google::GLOG_INFO:
      ROS_INFO_STREAM(message_string);
      break;

    case ::google::GLOG_WARNING:
      ROS_WARN_STREAM(message_string);
      break;

    case ::google::GLOG_ERROR:
      ROS_ERROR_STREAM(message_string);
      break;

    case ::google::GLOG_FATAL:
      ROS_FATAL_STREAM(message_string);
      will_die_ = true;
      break;
  }
}

void ScopedRosLogSink::WaitTillSent() 
{
    if (will_die_) {
        // Give ROS some time to actually publish our message.
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

}  // namespace vins_mono
}  // namespace cartographer_ros

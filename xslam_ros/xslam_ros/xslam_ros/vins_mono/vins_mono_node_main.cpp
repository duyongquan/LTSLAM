#include "xslam_ros/node.h"
#include "xslam_ros/ros_log_sink.h"

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tf2_ros/transform_listener.h"


DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");

DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");


namespace xslam_ros {
namespace {

void Run() 
{
    LOG(INFO) << "VINS Mono Start.";
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);


    std::cout << "--------------------------------";
    ::ros::spin();
}

}  // namespace
}  // namespace xslam_node

int main(int argc, char** argv) 
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty())
        << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";

    ::ros::init(argc, argv, "vins_mono_node");
    ::ros::start();

    xslam_ros::ScopedRosLogSink ros_log_sink;
    xslam_ros::Run();
    ::ros::shutdown();
}

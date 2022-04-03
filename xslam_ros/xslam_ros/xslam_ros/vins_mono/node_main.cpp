#include "xslam/vins/vins_builder.h"
#include "xslam_ros/vins_mono/node.h"
#include "xslam_ros/vins_mono/ros_log_sink.h"
#include "xslam_ros/vins_mono/node_options.h"

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

DEFINE_bool(
    start_default_topics, true,
    "Enable to immediately start the default topics.");

namespace xslam_ros {
namespace vins_mono {
namespace {

void Run() 
{
    LOG(INFO) << "VINS Mono Start.";
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);

    auto node_options = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    auto vio_builder = xslam::vins::CreateVinsBuilder(node_options.vins_builder_options);
    Node node(node_options, vio_builder, &tf_buffer);

    if (FLAGS_start_default_topics) {
        node.StartDefaultTopics();
    }
    
    ::ros::spin();
    node.RunFinalOptimization();
}

}  // namespace
}  // namespace vins_mono
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

    xslam_ros::vins_mono::ScopedRosLogSink ros_log_sink;
    xslam_ros::vins_mono::Run();
    ::ros::shutdown();
}

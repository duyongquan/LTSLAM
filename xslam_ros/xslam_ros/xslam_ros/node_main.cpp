#include "xslam_ros/node.h"

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tf2_ros/transform_listener.h"

namespace xslam_ros {
namespace {

void Run() 
{
    LOG(INFO) << "XSLAM Beginer.";
    std::cout << "--------------------------------";
    ::ros::spin();
}

}  // namespace
}  // namespace xslam_node

int main(int argc, char** argv) 
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);


    ::ros::init(argc, argv, "xslam_node");
    ::ros::start();

    xslam_ros::Run();
    ::ros::shutdown();
}

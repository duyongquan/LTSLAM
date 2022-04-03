#include "xslam/vins/vins.h"
#include "xslam/vins/common/version.h"
#include "xslam/vins/common/string.h"
#include "xslam/vins/common/logging.h"

constexpr const int kThreadNums = 4;

namespace xslam {
namespace vins {

VINSSystem::VINSSystem(const std::string& config_filename)
{
    ParseCommandLineFlags();

    pool_ = std::make_shared<common::ThreadPool>(kThreadNums);
    feature_tracker_ = std::make_shared<feature_tracker::FeatureTracker>(config_filename);
    pose_estimator_  = std::make_shared<estimator::Estimator>(config_filename);
    pose_graph_      = std::make_shared<pose_graph::PoseGraph>(config_filename);
}

VINSSystem::VINSSystem(const VINSOptions& options)
{

}

VINSSystem::VINSSystem(int argc, char **argv)
{
    ParseCommandLineFlags();
}

VINSSystem::~VINSSystem()
{

}

void VINSSystem::ShowHelp()
{
    std::cout << common::StringPrintf(
                   "%s -- VINS-Mono SLAM system.\n"
                   "              (%s)",
                   common::GetVersionInfo().c_str(), common::GetBuildInfo().c_str())
            << std::endl
            << std::endl;

    std::cout << "Usage:" << std::endl;
    std::cout << "  vins [command] [options]" << std::endl << std::endl;
    std::cout << "Example usage:" << std::endl;
    std::cout << "  vins help [ -h, --help ]" << std::endl;
    std::cout << "  ..." << std::endl << std::endl;

    std::cout << "Available commands:" << std::endl;
    std::cout << "  help" << std::endl;

}

void VINSSystem::HandleIMUSensorMessages(const sensor::ImuData& msg)
{

}


void VINSSystem::HandleImageSensorMessage(const sensor::ImageData& msg)
{

}

void VINSSystem::Shutdown()
{

}

void VINSSystem::ParseCommandLineFlags()
{

}


} // namespace vins
} // namespace xslam
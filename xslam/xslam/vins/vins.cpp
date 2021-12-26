#include "xslam/vins/vins.h"

namespace xslam {
namespace vins {

VINSSystem::VINSSystem(const std::string& config_filename)
{
    feature_tracker_ = std::make_shared<feature_tracker::FeatureTracker>(config_filename);
    pose_estimator_  = std::make_shared<estimator::Estimator>(config_filename);
    pose_graph_      = std::make_shared<pose_graph::PoseGraph>(config_filename);
}

VINSSystem::~VINSSystem()
{

}

void VINSSystem::HandleIMUSensorMessages()
{

}


void VINSSystem::HandleCameraSensorMessage()
{

}

} // namespace vins
} // namespace xslam
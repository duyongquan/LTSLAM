#include "xslam/vins/estimator/estimator.h"


namespace xslam {
namespace vins {
namespace estimator {
    

Estimator::Estimator(const std::string& filename)
{
}

ImuTracker* Estimator::imu_tracker()
{
    return imu_tracker_.get();
}

feature_tracker::FeatureTracker* Estimator::feature_tracker()
{
    return feature_tracker_.get();
}

} // namespace estimator
} // namespace vins
} // namespace xslam 

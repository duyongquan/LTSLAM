#include "xslam/vins/estimator/estimator.h"


namespace xslam {
namespace vins {
namespace estimator {

Estimator::Estimator(const proto::EstimatorOptions &options)
  : options_(options)
{
    // Thread pool
    pool_ = std::make_shared<common::ThreadPool>(4);

    // image
    feature_tracker_ = std::make_shared<feature_tracker::FeatureTracker>(
        options_.feature_tracker_options(), pool_.get());

    // IMU
    imu_tracker_ = std::make_shared<imu_tracker::ImuTracker>(
        options_.imu_tracker_options(), pool_.get());
}

imu_tracker::ImuTracker* Estimator::imu_tracker()
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

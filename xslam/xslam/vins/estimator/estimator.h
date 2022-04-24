#ifndef XSLAM_VINS_ESTIMATOR_ESTIMATOR_H
#define XSLAM_VINS_ESTIMATOR_ESTIMATOR_H

#include "xslam/vins/common/thread_pool.h"
#include "xslam/vins/estimator/imu_tracker.h"
#include "xslam/vins/feature_tracker/feature_tracker.h"
#include "xslam/vins/estimator/proto/estimator_options.pb.h"

#include <thread>
#include <string>
#include <mutex>
#include <memory>

namespace xslam {
namespace vins {
namespace estimator {
    
class Estimator 
{
public:
    explicit Estimator(const proto::EstimatorOptions &options);
    ImuTracker* imu_tracker();
    feature_tracker::FeatureTracker* feature_tracker();

private:
    proto::EstimatorOptions options_;
    std::shared_ptr<common::ThreadPool> pool_;
    std::shared_ptr<ImuTracker> imu_tracker_;
    std::shared_ptr<feature_tracker::FeatureTracker> feature_tracker_;
};

} // namespace estimator
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_ESTIMATOR_ESTIMATOR_H
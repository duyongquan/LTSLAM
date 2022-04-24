#ifndef XSLAM_VINS_ESTIMATOR_IMU_TRACKER_H
#define XSLAM_VINS_ESTIMATOR_IMU_TRACKER_H

#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/common/thread_pool.h"
#include "xslam/vins/estimator/proto/imu_tracker_options.pb.h"

#include <deque>

namespace xslam {
namespace vins {
namespace estimator {

class ImuTracker
{
public:
    explicit ImuTracker(const proto::ImuTrackerOptions& options, common::ThreadPool* pool);
    ~ImuTracker();

    ImuTracker(const ImuTracker&) = delete;
    ImuTracker& operator=(const ImuTracker&) = delete;

    void AddImuData(const sensor::ImuData& msg);

private:
    void DebugOptionsString();

    common::ThreadPool *thread_pool_;
    const proto::ImuTrackerOptions options_;
};

} // namespace estimator
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_ESTIMATOR_IMU_TRACKER_H

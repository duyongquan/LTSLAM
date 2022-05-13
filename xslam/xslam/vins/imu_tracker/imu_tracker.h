#ifndef XSLAM_VINS_ESTIMATOR_IMU_TRACKER_H
#define XSLAM_VINS_ESTIMATOR_IMU_TRACKER_H

#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/common/thread_pool.h"
#include "xslam/vins/imu_tracker/proto/imu_tracker_options.pb.h"

#include <deque>

namespace xslam {
namespace vins {
namespace imu_tracker {

class ImuTracker
{
public:
    explicit ImuTracker(const proto::ImuTrackerOptions& options, common::ThreadPool* pool);
    ~ImuTracker();

    ImuTracker(const ImuTracker&) = delete;
    ImuTracker& operator=(const ImuTracker&) = delete;

    void AddImuData(const sensor::ImuData& msg);

private:
    common::ThreadPool *thread_pool_;
    const proto::ImuTrackerOptions options_;
};

} // namespace imu_tracker
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_ESTIMATOR_IMU_TRACKER_H

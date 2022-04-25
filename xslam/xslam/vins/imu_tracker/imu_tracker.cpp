#include "xslam/vins/imu_tracker/imu_tracker.h"

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace imu_tracker {

ImuTracker::ImuTracker(const proto::ImuTrackerOptions& options, common::ThreadPool* pool)
  : thread_pool_(pool)
{
    DebugOptionsString();
}

ImuTracker::~ImuTracker()
{

}

void ImuTracker::AddImuData(const sensor::ImuData& msg)
{

}

void ImuTracker::DebugOptionsString()
{   
    LOG(INFO) << "acc_n: " << options_.acceleration_n();
    LOG(INFO) << "acc_w: " << options_.acceleration_w();
    LOG(INFO) << "gyro_n: " << options_.gyro_n();
    LOG(INFO) << "gyro_w: " << options_.gyro_w();
}

} // namespace imu_tracker
} // namespace vins
} // namespace xslam 
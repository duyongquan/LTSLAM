#include "xslam/vins/imu_tracker/imu_tracker.h"

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace imu_tracker {

ImuTracker::ImuTracker(const proto::ImuTrackerOptions& options, common::ThreadPool* pool)
  : thread_pool_(pool)
{

}

ImuTracker::~ImuTracker()
{

}

void ImuTracker::AddImuData(const sensor::ImuData& msg)
{
    LOG(INFO) << "ImuTracker::AddImuData";
}

} // namespace imu_tracker
} // namespace vins
} // namespace xslam 
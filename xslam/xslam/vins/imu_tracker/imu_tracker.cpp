#include "xslam/vins/imu_tracker/imu_tracker.h"

namespace xslam {
namespace vins {
namespace imu_tracker {

ImuTracker::ImuTracker(const proto::ImuTrackerOptions& options, common::ThreadPool* pool)
  : thread_pool_(pool)
{
    if (options_.print_options()) {
       DebugOptionsString();
    }
}

ImuTracker::~ImuTracker()
{

}

void ImuTracker::AddImuData(const sensor::ImuData& msg)
{

}

void ImuTracker::DebugOptionsString()
{
  
}

} // namespace imu_tracker
} // namespace vins
} // namespace xslam 
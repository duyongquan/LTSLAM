#include "xslam/vins/estimator/imu_tracker.h"

namespace xslam {
namespace vins {
namespace estimator {

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

} // namespace estimator
} // namespace vins
} // namespace xslam 
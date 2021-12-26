#include "xslam/vins/feature_tracker/feature_tracker.h"

#include <chrono>

namespace xslam {
namespace feature_tracker {
    

FeatureTracker::FeatureTracker(const std::string& filename)
{
}

FeatureTracker::FeatureTracker(const std::string& filename, common::ThreadPool* pool)
    : thread_pool_(pool)
{
}

void FeatureTracker::AddCameraData(const sensor::CameraData& cemara_data)
{

}

void FeatureTracker::RunTask()
{
    while (1) {
        LOG(INFO) << "Run task .....";
        sleep(1);
    }
}

void FeatureTracker::AddWorkItem(const common::Task::WorkItem& work_item)
{
    
}

} // namespace feature_tracker
} // namespace xslam 

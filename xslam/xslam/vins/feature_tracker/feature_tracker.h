#ifndef XSLAM_VINS_FEATURE_TRACKER_H
#define XSLAM_VINS_FEATURE_TRACKER_H

#include "xslam/vins/common/task.h"
#include "xslam/vins/common/thread_pool.h"
#include "xslam/vins/sensor/camera_data.h"

#include <thread>
#include <string>

namespace xslam {
namespace feature_tracker {
    
class FeatureTracker 
{
public:
    FeatureTracker(const std::string& filename);
    FeatureTracker(const std::string& filename, common::ThreadPool* pool);

    void AddCameraData(const sensor::CameraData& cemara_data);

    void RunTask();

private:
    // Handles a new work item.
    void AddWorkItem(const common::Task::WorkItem& work_item);

    common::ThreadPool *thread_pool_;
};

} // namespace feature_tracker
} // namespace xslam 

#endif // XSLAM_VINS_FEATURE_TRACKER_H
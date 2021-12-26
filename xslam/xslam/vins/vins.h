#ifndef XSLAM_VINS_VINS_H
#define XSLAM_VINS_VINS_H

#include "xslam/vins/feature_tracker/feature_tracker.h"
#include "xslam/vins/estimator/estimator.h"
#include "xslam/vins/pose_graph/pose_graph.h"

#include <map>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <iostream>

namespace xslam {
namespace vins {

class VINSSystem
{
public:
    VINSSystem(const std::string& config_filename);
    ~VINSSystem(); 

    VINSSystem(const VINSSystem& other) = delete;
    VINSSystem& operator=(const VINSSystem& ohter) = delete;

    // IMU sensor data
    void HandleIMUSensorMessages();

    // camera sensor data
    void HandleCameraSensorMessage();

private:
    std::shared_ptr<feature_tracker::FeatureTracker> feature_tracker_;
    std::shared_ptr<estimator::Estimator> pose_estimator_;
    std::shared_ptr<pose_graph::PoseGraph> pose_graph_;
};

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_H
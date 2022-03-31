#ifndef XSLAM_VINS_VINS_H
#define XSLAM_VINS_VINS_H

#include "xslam/vins/feature_tracker/feature_tracker.h"
#include "xslam/vins/estimator/estimator.h"
#include "xslam/vins/pose_graph/pose_graph.h"
#include "xslam/vins/common/thread_pool.h"
#include "xslam/vins/sensor/image_data.h"
#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/vins_options.h"

#include <map>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <iostream>
#include <functional>

namespace xslam {
namespace vins {


class VINSSystem
{
public:
    using CommandFunction = std::function<int(int, char**)>;

    VINSSystem(const std::string& config_filename);
    VINSSystem(const VINSOptions& options);
    VINSSystem(int argc, char **argv);

    ~VINSSystem(); 

    VINSSystem(const VINSSystem& other) = delete;
    VINSSystem& operator=(const VINSSystem& ohter) = delete;

    // Show information: feature, estimator ans pose graph command.
    void ShowHelp();

    // IMU sensor data
    void HandleIMUSensorMessages(const sensor::ImuData& msg);

    // camera sensor data
    void HandleImageSensorMessage(const sensor::ImageData& msg);

    // Shutdown all thread
    void Shutdown();

private:
    // Load subsystem commands
    void ParseCommandLineFlags();

    std::shared_ptr<feature_tracker::FeatureTracker> feature_tracker_;
    std::shared_ptr<estimator::Estimator> pose_estimator_;
    std::shared_ptr<pose_graph::PoseGraph> pose_graph_;

    // thread pools
    std::shared_ptr<common::ThreadPool> pool_;

    // All subsystem command
    std::vector<std::pair<std::string, VINSSystem::CommandFunction>> commands_;
};

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_H
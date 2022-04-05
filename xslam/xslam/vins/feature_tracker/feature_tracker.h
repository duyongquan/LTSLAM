#ifndef XSLAM_VINS_FEATURE_TRACKER_H
#define XSLAM_VINS_FEATURE_TRACKER_H

#include "xslam/vins/common/port.h"
#include "xslam/vins/common/task.h"
#include "xslam/vins/common/thread_pool.h"
#include "xslam/vins/sensor/image_data.h"
#include "xslam/vins/feature_tracker/proto/feature_tracker_options.pb.h"
#include "xslam/vins/camera/pinhole_camera.h"

#include <thread>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace xslam {
namespace vins {
namespace feature_tracker {

class FeatureTracker 
{
public:
    explicit FeatureTracker(const proto::FeatureTrackerOptions &options);
    FeatureTracker(const std::string& filename);
    FeatureTracker(const std::string& filename, common::ThreadPool* pool);
    FeatureTracker() {}

    void AddImageData(const sensor::ImageData& image);

    void SetMask();

    void AddPoints();

    bool UpdateID(uint8 id);

    void ShowUndistortion(const std::string &name);

    void RejectWithF();

    void UndistortedPoints();

    void RunTask();

private:
    // Handles a new work item.
    void AddWorkItem(const common::Task::WorkItem& work_item);

    bool CheckInBorder(const cv::Point2f& point);

    void ReduceVector(std::vector<cv::Point2f>& v, std::vector<uchar> status);

    void ReduceVector(std::vector<int> &v, std::vector<uchar> status);

    common::ThreadPool *thread_pool_;
    const proto::FeatureTrackerOptions options_;

    cv::Mat mask_;
    cv::Mat fisheye_mask_;
    cv::Mat prev_img, cur_img, forw_img;
    std::vector<cv::Point2f> n_pts;
    std::vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    std::vector<cv::Point2f> prev_un_pts, cur_un_pts;
    std::vector<cv::Point2f> pts_velocity;
    std::vector<int> ids;
    std::vector<int> track_cnt;
    std::map<int, cv::Point2f> cur_un_pts_map;
    std::map<int, cv::Point2f> prev_un_pts_map;

    std::unique_ptr<camera::PinholeCamera> camera_;
    double cur_time;
    double prev_time;

    inline static int n_id = 0;
};

std::unique_ptr<FeatureTracker> CreateFeatureTracker(
    const proto::FeatureTrackerOptions& options);

} // namespace feature_tracker
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_FEATURE_TRACKER_H
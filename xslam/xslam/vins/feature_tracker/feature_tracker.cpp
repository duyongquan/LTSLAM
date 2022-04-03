#include "xslam/vins/feature_tracker/feature_tracker.h"

#include <chrono>

namespace xslam {
namespace vins {
namespace feature_tracker {

FeatureTracker::FeatureTracker(const proto::FeatureTrackerOptions &options)
    : options_(options)
{

}

FeatureTracker::FeatureTracker(const std::string& filename)
{
}

FeatureTracker::FeatureTracker(const std::string& filename, common::ThreadPool* pool)
    : thread_pool_(pool)
{
}

void FeatureTracker::AddImageData(const sensor::ImageData& image)
{
   
        
}

void FeatureTracker::SetMask()
{
    if(options_.fisheye()) {
        mask_ = fisheye_mask_.clone();
    } 

    mask_ = cv::Mat(options_.row(), options_.col(), CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++) {
        cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(forw_pts[i], ids[i])));
    }
        
    std::sort(cnt_pts_id.begin(), cnt_pts_id.end(), 
        [](const std::pair<int, std::pair<cv::Point2f, int>> &a, 
           const std::pair<int, std::pair<cv::Point2f, int>> &b) {
        return a.first > b.first;
    });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask_.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask_, it.second.first, options_.min_distance(), 0, -1);
        }
    }
}

void FeatureTracker::AddPoints()
{
    for (auto &p : n_pts) {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

bool FeatureTracker::UpdateID(uint8 id)
{
    if (id < ids.size()) {
        if (ids[id] == -1) {
            ids[id] = n_id++;
        }
        return true;
    }
    return false;
}

void FeatureTracker::ShowUndistortion(const std::string &name)
{
    cv::Mat undistortedImg(options_.row() + 600, options_.col() + 600, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < options_.col(); i++) {
        for (int j = 0; j < options_.row(); j++) {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            // m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
        }
    }

    for (int i = 0; i < undistortedp.size(); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * options_.focal_length() + options_.col() / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * options_.focal_length() + options_.row() / 2;
        pp.at<float>(2, 0) = 1.0;

        if (pp.at<float>(1, 0) + 300 >= 0 && 
            pp.at<float>(1, 0) + 300 < options_.row() + 600 && 
            pp.at<float>(0, 0) + 300 >= 0 && 
            pp.at<float>(0, 0) + 300 < options_.col() + 600) {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = 
                cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
        
}

void FeatureTracker::RejectWithF()
{
    if (forw_pts.size() < 8) {
        return;
    }

    LOG(INFO) << "FM ransac begins";
    std::vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector3d tmp_p;
        // m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
        tmp_p.x() = options_.focal_length()  * tmp_p.x() / tmp_p.z() + options_.col() / 2.0;
        tmp_p.y() = options_.focal_length()  * tmp_p.y() / tmp_p.z() + options_.row() / 2.0;
        un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

        // m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
        tmp_p.x() = options_.focal_length()  * tmp_p.x() / tmp_p.z() + options_.col() / 2.0;
        tmp_p.y() = options_.focal_length()  * tmp_p.y() / tmp_p.z() + options_.row() / 2.0;
        un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    std::vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, options_.fundamental_threshold(), 0.99, status);
    int size_a = cur_pts.size();
    ReduceVector(prev_pts, status);
    ReduceVector(cur_pts, status);
    ReduceVector(forw_pts, status);
    ReduceVector(cur_un_pts, status);
    ReduceVector(ids, status);
    ReduceVector(track_cnt, status);
    // INFO("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
}

void FeatureTracker::UndistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        // m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(std::make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    }

    // Caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++) {
            if (ids[i] != -1)
            {
                auto it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end()) {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else {
                     pts_velocity.push_back(cv::Point2f(0, 0));
                }
                   
            }
            else {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    } else {
        for (unsigned int i = 0; i < cur_pts.size(); i++) {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
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

bool FeatureTracker::CheckInBorder(const cv::Point2f& point)
{
  const int kBorderSize = 1;
  int img_x = cvRound(point.x);
  int img_y = cvRound(point.y);

  return kBorderSize <= img_x && img_x < options_.col() - kBorderSize && 
         kBorderSize <= img_y && img_y < options_.row() - kBorderSize;
}

void FeatureTracker::ReduceVector(std::vector<cv::Point2f>& v, const std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++) {
        if (status[i]) {
            v[j++] = v[i];
        } 
    }
    v.resize(j);
}

void FeatureTracker::ReduceVector(std::vector<int> &v, const std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++) {
        if (status[i]) {
            v[j++] = v[i];
        }
    }     
    v.resize(j);
}

} // namespace feature_tracker
} // namespace vins
} // namespace xslam 

#include "xslam/vins/feature_tracker/feature_tracker.h"
#include "glog/logging.h"

#include <chrono>
#include <vector>
#include <set>

namespace xslam {
namespace vins {
namespace feature_tracker {

FeatureTracker::FeatureTracker(
    const proto::FeatureTrackerOptions &options,
    common::ThreadPool* pool)
    : options_(options),
      thread_pool_(pool)
{
    camera_ = std::make_unique<camera::PinholeCamera>(options);
}

void FeatureTracker::AddImageData(const sensor::ImageData& image)
{
    if(first_image_)
    {
        first_image_ = false;
        first_image_time_ = image.time;
        last_image_time_ = image.time;
        return;
    }

    // detect unstable camera stream
    const double delta_t = common::ToSeconds(image.time - last_image_time_);
    if (delta_t > 1.0 || image.time < last_image_time_)
    {
        LOG(WARNING) << "image discontinue! reset the feature tracker!";
        first_image_ = true; 
        last_image_time_ = common::Time::min();
        pub_count = 1;
        restart_ = true;
        return;
    }

    last_image_time_ = image.time;
    const int frequency = round(1.0 * pub_count / common::ToSeconds(image.time - first_image_time_));
    // frequency control
    if (frequency <= options_.freq())
    {
        pub_this_frame_ = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (common::ToSeconds(image.time - first_image_time_) - options_.freq()) < 0.01 * options_.freq()))
        {
            first_image_time_ = image.time;
            pub_count = 0;
        }
    }
    else
        pub_this_frame_ = false;

    HandleImageMessages(image.image);

    for (unsigned int i = 0;; i++)
    {
        bool completed = UpdateID(i);
        if (!completed)
            break;
    }

    if (!pub_this_frame_)
    {   
        LOG(INFO) << "Not publish this image frame.";
        return;
    }

    pub_count++;
    FeaturePoints feature_points;
    common::messages::ChannelFloat32 id_of_point;
    common::messages::ChannelFloat32 u_of_point;
    common::messages::ChannelFloat32 v_of_point;
    common::messages::ChannelFloat32 velocity_x_of_point;
    common::messages::ChannelFloat32 velocity_y_of_point;
    
    for (unsigned int j = 0; j < ids.size(); j++)
    {
        if (track_cnt[j] > 1)
        {
            int p_id = ids[j];
            common::messages::Point32 p;
            p.x = cur_un_pts[j].x;
            p.y = cur_un_pts[j].y;
            p.z = 1;

            feature_points.points.push_back(p);
            id_of_point.values.push_back(p_id);
            u_of_point.values.push_back(cur_pts[j].x);
            v_of_point.values.push_back(cur_pts[j].y);
            velocity_x_of_point.values.push_back(pts_velocity[j].x);
            velocity_y_of_point.values.push_back(pts_velocity[j].y);
        }
    }
  
    feature_points.channels.push_back(id_of_point);
    feature_points.channels.push_back(u_of_point);
    feature_points.channels.push_back(v_of_point);
    feature_points.channels.push_back(velocity_x_of_point);
    feature_points.channels.push_back(velocity_y_of_point);
    queue_.push_back(feature_points);

    if (!options_.show_track()) {
      return;
    }
}

bool FeatureTracker::CheckRestart()
{
    common::MutexLocker locker(&mutex_);
    return restart_;
}

void FeatureTracker::HandleImageMessages(const cv::Mat& current_image)
{
    cur_img = current_image;
    cv::Mat img;

    if (options_.equalize())
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(current_image, img);
    }
    else
        img = current_image;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();
    if (cur_pts.size() > 0)
    {
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++) {
            if (status[i] && !CheckInBorder(forw_pts[i])) {
                status[i] = 0;
            }
        }
        
        ReduceVector(prev_pts, status);
        ReduceVector(cur_pts, status);
        ReduceVector(forw_pts, status);
        ReduceVector(ids, status);
        ReduceVector(cur_un_pts, status);
        ReduceVector(track_cnt, status);
    }

    for (auto &n : track_cnt) {
        n++;
    }

    if (pub_this_frame_) 
    {
        RejectWithF();
        LOG(INFO) << "Set mask begins";
        SetMask();
      
        LOG(INFO) << "detect feature begins";
  
        int n_max_cnt = options_.max_cnt() - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask_.empty())
                LOG(INFO) << "mask is empty ";
            if (mask_.type() != CV_8UC1)
                LOG(INFO) << "mask type wrong ";
            if (mask_.size() != forw_img.size())
                LOG(INFO) << "wrong size ";
            cv::goodFeaturesToTrack(forw_img, n_pts, options_.max_cnt() - forw_pts.size(), 
                0.01, options_.min_distance(), mask_);
        }
        else
            n_pts.clear();
       
        LOG(INFO) << "Add feature begins";
        AddPoints();
    } 

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    UndistortedPoints();
    prev_time = cur_time;
}

bool FeatureTracker::GetNewestFeaturePoints(FeaturePoints& points)
{
    if (queue_.empty()) {
      return false;
    }

    points = queue_.front();
    queue_.pop_front();
    return true;
}

void FeatureTracker::SetMask()
{
    if(options_.fisheye()) {
        mask_ = fisheye_mask_.clone();
    } 
    mask_ = cv::Mat(options_.image_height(), options_.image_width(), CV_8UC1, cv::Scalar(255));

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
    cv::Mat undistortedImg(options_.image_height() + 600, options_.image_width() + 600, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < options_.image_height(); i++) {
        for (int j = 0; j < options_.image_width(); j++) {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            camera_->LiftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
        }
    }

    for (int i = 0; i < undistortedp.size(); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * options_.focal_length() + options_.image_height() / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * options_.focal_length() + options_.image_width() / 2;
        pp.at<float>(2, 0) = 1.0;

        if (pp.at<float>(1, 0) + 300 >= 0 && 
            pp.at<float>(1, 0) + 300 < options_.image_height() + 600 && 
            pp.at<float>(0, 0) + 300 >= 0 && 
            pp.at<float>(0, 0) + 300 < options_.image_width() + 600) {
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
        camera_->LiftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
        tmp_p.x() = options_.focal_length()  * tmp_p.x() / tmp_p.z() + options_.image_height() / 2.0;
        tmp_p.y() = options_.focal_length()  * tmp_p.y() / tmp_p.z() + options_.image_width() / 2.0;
        un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

        camera_->LiftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
        tmp_p.x() = options_.focal_length()  * tmp_p.x() / tmp_p.z() + options_.image_height() / 2.0;
        tmp_p.y() = options_.focal_length()  * tmp_p.y() / tmp_p.z() + options_.image_width() / 2.0;
        un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    std::vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, options_.f_threshold(), 0.99, status);
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
        camera_->LiftProjective(a, b);
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

bool FeatureTracker::CheckInBorder(const cv::Point2f& point)
{
  const int kBorderSize = 1;
  int img_x = cvRound(point.x);
  int img_y = cvRound(point.y);

  return kBorderSize <= img_x && img_x < options_.image_height() - kBorderSize && 
         kBorderSize <= img_y && img_y < options_.image_width() - kBorderSize;
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

void FeatureTracker::DebugOptionsString()
{

}

} // namespace feature_tracker
} // namespace vins
} // namespace xslam 

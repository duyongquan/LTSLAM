#ifndef XSLAM_ROS_SLAM2D_SLAM2D_H
#define XSLAM_ROS_SLAM2D_SLAM2D_H

#include "xslam_ros/slam2d/types.h"

#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include <opencv2/opencv.hpp>

namespace xslam_ros {
namespace slam2d {

class Slam2D
{
public:
    Slam2D();

    void HandleLaserScanMessages(const sensor_msgs::MultiEchoLaserScanConstPtr &msg);
    void HandleLaserScanMessages(const sensor_msgs::LaserScanConstPtr &msg);

    void ScanMatch();
    void MatchRandom();
    int  MatchScore(Eigen::Vector3d pose);
    void Update();
    void UpdateTransform();

    void Bresenham(cv::Point2i p1, cv::Point2i p2);
    void UpdateMap();
    void CVMapToMap();//convert cv map to map
    double timestamp() const;
    State2D state() const;
    void set_map2d_timestamp(ros::Time timestamp);

    nav_msgs::OccupancyGrid& GetOccupancyGridMap();

private:
    Eigen::Vector2d WorldToMap(Eigen::Vector2d p);
    cv::Point2i WorldToMap(cv::Point2f p);


    State2D state_;
    State2D delta_;

    double timestamp_;
    nav_msgs::OccupancyGrid map2d_;
    cv::Mat cvmap2d_;

    pcl::PointCloud<PointType> scan_;
    pcl::PointCloud<PointType> scan_prev_;

    bool cvmap_vis_enable_ = false;
};

} // namespace slam2d
} // namespace xslam_ros

#endif // XSLAM_ROS_SLAM2D_SLAM2D_H


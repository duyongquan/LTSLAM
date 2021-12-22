#ifndef XSLAM_ROS_SLAM2D_TYPES_H
#define XSLAM_ROS_SLAM2D_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Eigen>

namespace xslam_ros {
namespace slam2d {

typedef pcl::PointXY PointType;

struct State2D
{
    double theta;
    Eigen::Vector2d t;
};
    
Eigen::Vector2d ToEigen(PointType point);
PointType ToPoint(Eigen::Vector2d pose);

} // namespace slam2d
} // namespace xslam_ros

#endif // XSLAM_ROS_SLAM2D_TYPES_H

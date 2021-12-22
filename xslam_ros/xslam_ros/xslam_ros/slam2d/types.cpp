#include "xslam_ros/slam2d/types.h"

namespace xslam_ros {
namespace slam2d {

Eigen::Vector2d ToEigen(PointType point)
{
    return {
        point.x, 
        point.y
    };
}

PointType ToPoint(Eigen::Vector2d pose)
{
    return {
        pose(0),
        pose(1)
    };
}

} // namespace slam2d
} // namespace xslam_ros

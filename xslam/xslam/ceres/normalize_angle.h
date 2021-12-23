//
// Created by quan on 2021/12/10.
//

#ifndef SLAM_NORMALIZE_ANGLE_H
#define SLAM_NORMALIZE_ANGLE_H

#include <cmath>

#include "ceres/ceres.h"

namespace xslam {
namespace ceres {
namespace example {

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T& angle_radians)
{
    // Use ceres::floor because it is specialized for double and Jet types.
    T two_pi(2.0 * M_PI);
    return angle_radians - two_pi * ::ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

} // namespace example
} // namespace ceres
} // namespace slam

#endif //SLAM_NORMALIZE_ANGLE_H

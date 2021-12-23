//
// Created by quan on 2021/12/7.
//

#ifndef SLAM_POWELL_H
#define SLAM_POWELL_H

#include "ceres/ceres.h"
#include "glog/logging.h"

namespace xslam {
namespace ceres {

struct F1
{
    template<typename T>
    bool operator()(const T *const x1, const T *const x2, T *residual) const {
        // f1 = x1 + 10 * x2;
        residual[0] = x1[0] + 10.0 * x2[0];
        return true;
    }
};

struct F2
{
    template<typename T>
    bool operator()(const T *const x3, const T *const x4, T *residual) const {
        // f2 = sqrt(5) (x3 - x4)
        residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
        return true;
    }
};

struct F3
{
    template<typename T>
    bool operator()(const T *const x2, const T *const x3, T *residual) const {
        // f3 = (x2 - 2 x3)^2
        residual[0] = (x2[0] - 2.0 * x3[0]) * (x2[0] - 2.0 * x3[0]);
        return true;
    }
};

struct F4
{
    template<typename T>
    bool operator()(const T *const x1, const T *const x4, T *residual) const {
        // f4 = sqrt(10) (x1 - x4)^2
        residual[0] = sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
        return true;
    }
};

class Powell
{
public:
    void RunDemo();
};

}
}

#endif //SLAM_POWELL_H

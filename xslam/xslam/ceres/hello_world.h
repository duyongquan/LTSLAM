//
// Created by quan on 2021/12/6.
//

#ifndef SLAM_HELLO_WORLD_H
#define SLAM_HELLO_WORLD_H

#include "ceres/ceres.h"
#include "glog/logging.h"

namespace xslam {
namespace ceres {

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        // min f(x) = (10 - x)
        residual[0] = 10.0 - x[0];
        return true;
    }
};

class HelloWorld
{
public:
    void RunDemo();
};

} // namespace ceres
} // namespace xslam
#endif //SLAM_HELLO_WORLD_H

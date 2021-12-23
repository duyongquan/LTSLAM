//
// Created by quan on 2021/12/9.
//

#ifndef SLAM_POSE_GRAPH_2D_H
#define SLAM_POSE_GRAPH_2D_H

#include "xslam/ceres/pose_data_types.h"
#include "xslam/ceres/pose_graph_2d_error_term.h"

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "glog/logging.h"

namespace xslam {
namespace ceres {
namespace example {

class PoseGraph2D
{
public:
    void RunSLAM();

private:
    void BuildOptimizationProblem(const std::vector<Constraint2d> &constraints,
                                  std::map<int, Pose2d> *poses,
                                  ::ceres::Problem *problem);

    bool SolveOptimizationProblem(::ceres::Problem *problem);


    bool OutputPoses(const std::string &filename,
                     const std::map<int, Pose2d> &poses);

};

} // namespace example
} // namespace ceres
} // namespace xslam

#endif //SLAM_POSE_GRAPH_2D_H

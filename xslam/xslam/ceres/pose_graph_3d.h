//
// Created by quan on 2021/12/10.
//

#ifndef SLAM_POSE_GRAPH_3D_H
#define SLAM_POSE_GRAPH_3D_H

#include "xslam/ceres/pose_graph_3d_error_term.h"
#include "xslam/ceres/read_g2o.h"
#include "xslam/ceres/pose_data_types.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

#include <iostream>
#include <fstream>
#include <string>


namespace xslam {
namespace ceres {
namespace example {

class PoseGraph3D
{
public:
    void RunSLAM();

private:
    // Constructs the nonlinear least squares optimization problem from the pose
    // graph constraints.
    void BuildOptimizationProblem(const VectorOfConstraints& constraints,
                                  MapOfPoses* poses, ::ceres::Problem* problem);

    // Returns true if the solve was successful.
    bool SolveOptimizationProblem(::ceres::Problem* problem);

    // Output the poses to the file with format: id x y z q_x q_y q_z q_w.
    bool OutputPoses(const std::string& filename, const MapOfPoses& poses);

};

} // namespace example
} // namespace ceres
} // namespace xslam

#endif //SLAM_POSE_GRAPH_3D_H

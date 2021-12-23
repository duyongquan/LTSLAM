//
// Created by quan on 2021/12/9.
//

#include "xslam/ceres/utils.h"
#include "xslam/ceres/read_g2o.h"
#include "xslam/ceres/pose_graph_2d.h"
#include "xslam/ceres/angle_local_parameterization.h"

namespace xslam {
namespace ceres {
namespace example {


void PoseGraph2D::RunSLAM() {
    std::map<int, Pose2d> poses;
    std::vector<Constraint2d> constraints;

    // The pose graph definition filename in g2o format.
    std::string input_filename =  GetCeresDatasetDirectory() + "M3500.g2o";

    CHECK(ReadG2oFile(input_filename, &poses, &constraints))
                    << "Error reading the file: " << input_filename;

    std::cout << "Number of poses: " << poses.size() << '\n';
    std::cout << "Number of constraints: " << constraints.size() << '\n';

    CHECK(OutputPoses("poses_original.txt", poses))
                    << "Error outputting to poses_original.txt";

    ::ceres::Problem problem;
    BuildOptimizationProblem(constraints, &poses, &problem);

    CHECK(SolveOptimizationProblem(&problem))
                    << "The solve was not successful, exiting.";

    CHECK(OutputPoses("poses_optimized.txt", poses))
                    << "Error outputting to poses_original.txt";
}


void PoseGraph2D::BuildOptimizationProblem(const std::vector<Constraint2d> &constraints,
                                           std::map<int, Pose2d> *poses,
                                           ::ceres::Problem *problem) {
    CHECK(poses != NULL);
    CHECK(problem != NULL);
    if (constraints.empty()) {
        LOG(INFO) << "No constraints, no problem to optimize.";
        return;
    }

    ::ceres::LossFunction* loss_function = NULL;
    ::ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

    for (std::vector<Constraint2d>::const_iterator constraints_iter = constraints.begin();
         constraints_iter != constraints.end(); ++constraints_iter) {
        const Constraint2d& constraint = *constraints_iter;

        std::map<int, Pose2d>::iterator pose_begin_iter = poses->find(constraint.id_begin);
        CHECK(pose_begin_iter != poses->end())
                        << "Pose with ID: " << constraint.id_begin << " not found.";
        std::map<int, Pose2d>::iterator pose_end_iter = poses->find(constraint.id_end);
        CHECK(pose_end_iter != poses->end())
                        << "Pose with ID: " << constraint.id_end << " not found.";

        const Eigen::Matrix3d sqrt_information = constraint.information.llt().matrixL();
        // Ceres will take ownership of the pointer.
        ::ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
                constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);
        problem->AddResidualBlock(
                cost_function, loss_function, &pose_begin_iter->second.x,
                &pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
                &pose_end_iter->second.x, &pose_end_iter->second.y,
                &pose_end_iter->second.yaw_radians);

        problem->SetParameterization(&pose_begin_iter->second.yaw_radians, angle_local_parameterization);
        problem->SetParameterization(&pose_end_iter->second.yaw_radians, angle_local_parameterization);
    }

    // The pose graph optimization problem has three DOFs that are not fully
    // constrained. This is typically referred to as gauge freedom. You can apply
    // a rigid body transformation to all the nodes and the optimization problem
    // will still have the exact same cost. The Levenberg-Marquardt algorithm has
    // internal damping which mitigate this issue, but it is better to properly
    // constrain the gauge freedom. This can be done by setting one of the poses
    // as constant so the optimizer cannot change it.
    std::map<int, Pose2d>::iterator pose_start_iter = poses->begin();
    CHECK(pose_start_iter != poses->end()) << "There are no poses.";
    problem->SetParameterBlockConstant(&pose_start_iter->second.x);
    problem->SetParameterBlockConstant(&pose_start_iter->second.y);
    problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

bool PoseGraph2D::SolveOptimizationProblem(::ceres::Problem *problem) {
    CHECK(problem != NULL);

    ::ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ::ceres::SPARSE_NORMAL_CHOLESKY;

    ::ceres::Solver::Summary summary;
    ::ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << '\n';
    return summary.IsSolutionUsable();
}

bool PoseGraph2D::OutputPoses(const std::string &filename,
                              const std::map<int, Pose2d> &poses) {
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
        std::cerr << "Error opening the file: " << filename << '\n';
        return false;
    }

    for (std::map<int, Pose2d>::const_iterator poses_iter = poses.begin();
         poses_iter != poses.end(); ++poses_iter)
    {
        const std::map<int, Pose2d>::value_type& pair = *poses_iter;
        outfile <<  pair.first << " " << pair.second.x << " " << pair.second.y
                << ' ' << pair.second.yaw_radians << '\n';
    }
    return true;
}

} // namespace example
} // namespace ceres
} // namespace xslam
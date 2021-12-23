//
// Created by quan on 2021/12/10.
//

#include "xslam/ceres/pose_graph_3d.h"
#include "xslam/ceres/utils.h"

namespace xslam {
namespace ceres {
namespace example {

void PoseGraph3D::RunSLAM()
{
    std::string input_filename =  GetCeresDatasetDirectory() + "sphere.g2o";

    CHECK(input_filename != "") << "Need to specify the filename to read.";

    MapOfPoses poses;
    VectorOfConstraints constraints;

    CHECK(ReadG2oFile(input_filename, &poses, &constraints))
                    << "Error reading the file: " << input_filename;

    std::cout << "Number of poses: " << poses.size() << '\n';
    std::cout << "Number of constraints: " << constraints.size() << '\n';

    CHECK(OutputPoses("poses_original_3d.txt", poses))
                    << "Error outputting to poses_original.txt";

    ::ceres::Problem problem;
    BuildOptimizationProblem(constraints, &poses, &problem);

    CHECK(SolveOptimizationProblem(&problem))
                    << "The solve was not successful, exiting.";

    CHECK(OutputPoses("poses_optimized_3d.txt", poses))
                    << "Error outputting to poses_original.txt";
}

void PoseGraph3D::BuildOptimizationProblem(
        const VectorOfConstraints& constraints,
        MapOfPoses* poses, ::ceres::Problem* problem)
{
    CHECK(poses != NULL);
    CHECK(problem != NULL);
    if (constraints.empty()) {
        LOG(INFO) << "No constraints, no problem to optimize.";
        return;
    }

    ::ceres::LossFunction* loss_function = NULL;
    ::ceres::LocalParameterization* quaternion_local_parameterization =
            new ::ceres::EigenQuaternionParameterization;

    for (VectorOfConstraints::const_iterator constraints_iter =
            constraints.begin();
         constraints_iter != constraints.end(); ++constraints_iter) {
        const Constraint3d& constraint = *constraints_iter;

        MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
        CHECK(pose_begin_iter != poses->end())
                        << "Pose with ID: " << constraint.id_begin << " not found.";
        MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
        CHECK(pose_end_iter != poses->end())
                        << "Pose with ID: " << constraint.id_end << " not found.";

        const Eigen::Matrix<double, 6, 6> sqrt_information =
                constraint.information.llt().matrixL();
        // Ceres will take ownership of the pointer.
        ::ceres::CostFunction* cost_function =
                PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

        problem->AddResidualBlock(cost_function, loss_function,
                                  pose_begin_iter->second.p.data(),
                                  pose_begin_iter->second.q.coeffs().data(),
                                  pose_end_iter->second.p.data(),
                                  pose_end_iter->second.q.coeffs().data());

        problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                     quaternion_local_parameterization);
        problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    // The pose graph optimization problem has six DOFs that are not fully
    // constrained. This is typically referred to as gauge freedom. You can apply
    // a rigid body transformation to all the nodes and the optimization problem
    // will still have the exact same cost. The Levenberg-Marquardt algorithm has
    // internal damping which mitigates this issue, but it is better to properly
    // constrain the gauge freedom. This can be done by setting one of the poses
    // as constant so the optimizer cannot change it.
    MapOfPoses::iterator pose_start_iter = poses->begin();
    CHECK(pose_start_iter != poses->end()) << "There are no poses.";
    problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
    problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

bool PoseGraph3D::SolveOptimizationProblem(::ceres::Problem* problem)
{
    CHECK(problem != NULL);

    ::ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ::ceres::SPARSE_NORMAL_CHOLESKY;

    ::ceres::Solver::Summary summary;
    ::ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << '\n';
    return summary.IsSolutionUsable();
}

bool PoseGraph3D::OutputPoses(const std::string& filename, const MapOfPoses& poses)
{
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
        LOG(ERROR) << "Error opening the file: " << filename;
        return false;
    }

    for (std::map<int, Pose3d, std::less<int>,
            Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
         const_iterator poses_iter = poses.begin();
         poses_iter != poses.end(); ++poses_iter) {
        const std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
        value_type& pair = *poses_iter;
        outfile << pair.first << " " << pair.second.p.transpose() << " "
                << pair.second.q.x() << " " << pair.second.q.y() << " "
                << pair.second.q.z() << " " << pair.second.q.w() << '\n';
    }
    return true;
}

} // namespace example
} // namespace ceres
} // namespace xslam
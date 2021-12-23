//
// Created by quan on 2021/12/9.
//

#include "xslam/ceres/simple_bundle_adjuster.h"
#include "xslam/ceres/utils.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

BALProblem::~BALProblem()
{
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
}

double* BALProblem::mutable_camera_for_observation(int i)
{
    return mutable_cameras() + camera_index_[i] * 9;
}

double* BALProblem::mutable_point_for_observation(int i)
{
    return mutable_points() + point_index_[i] * 3;
}

bool BALProblem::LoadFile(const char* filename)
{
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
        return false;
    };

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);
    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];
    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i)
    {
        FscanfOrDie(fptr, "%d", camera_index_ + i);
        FscanfOrDie(fptr, "%d", point_index_ + i);
        for (int j = 0; j < 2; ++j)
        {
            FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
        }
    }

    for (int i = 0; i < num_parameters_; ++i) {
        FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    return true;
}

void SimpleBundleAdjuster::RunDemo()
{
    BALProblem bal_problem;
    std::string kBALFilename = GetCeresDatasetDirectory() + "problem-318-41628-pre.txt";

    if (!bal_problem.LoadFile(kBALFilename.c_str())) {
        std::cerr << "ERROR: unable to open file " << kBALFilename << "\n";
        return;
    }

    const double* observations = bal_problem.observations();
    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ::ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        ::ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(
                observations[2 * i + 0], observations[2 * i + 1]);
        problem.AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    // Ax = b
    ::ceres::Solver::Options options;
    options.linear_solver_type = ::ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ::ceres::Solver::Summary summary;
    ::ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

} // namespace ceres
} // namespace xslam

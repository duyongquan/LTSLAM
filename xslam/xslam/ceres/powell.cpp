//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/powell.h"

namespace xslam {
namespace ceres {

void Powell::RunDemo()
{
    double x1 = 3.0;
    double x2 = -1.0;
    double x3 = 0.0;
    double x4 = 1.0;

    ::ceres::Problem problem;
    // Add residual terms to the problem using the autodiff
    // wrapper to get the derivatives automatically. The parameters, x1 through
    // x4, are modified in place.
    problem.AddResidualBlock(
            new ::ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1), NULL, &x1, &x2);
    problem.AddResidualBlock(
            new ::ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2), NULL, &x3, &x4);
    problem.AddResidualBlock(
            new ::ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3), NULL, &x2, &x3);
    problem.AddResidualBlock(
            new ::ceres::AutoDiffCostFunction<F4, 1, 1, 1>(new F4), NULL, &x1, &x4);

    ::ceres::Solver::Options options;
//    LOG_IF(FATAL, !::ceres::StringToMinimizerType(CERES_GET_FLAG(FLAGS_minimizer),
//                                                &options.minimizer_type))
//                    << "Invalid minimizer: " << CERES_GET_FLAG(FLAGS_minimizer)
//                    << ", valid options are: trust_region and line_search.";

    options.max_num_iterations = 100;
    options.linear_solver_type = ::ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    // clang-format off
    std::cout << "Initial x1 = " << x1
              << ", x2 = " << x2
              << ", x3 = " << x3
              << ", x4 = " << x4
              << "\n";
    // clang-format on
    // Run the solver!
    ::ceres::Solver::Summary summary;
    ::ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    // clang-format off
    std::cout << "Final x1 = " << x1
              << ", x2 = " << x2
              << ", x3 = " << x3
              << ", x4 = " << x4
              << "\n";

}

}
}

//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/helloworld_analytic_diff.h"

namespace xslam {
namespace ceres {

void AnalyticDiff::RunDemo()
{
    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;

    // Build the problem.
    ::ceres::Problem problem;

    // Set up the only cost function (also known as residual).
    ::ceres::CostFunction* cost_function = new QuadraticCostFunction;
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    // SVD
    // QR
    // PCG
    // Ax = b
    ::ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;

    ::ceres::Solver::Summary summary;
    ::ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
}

}
}
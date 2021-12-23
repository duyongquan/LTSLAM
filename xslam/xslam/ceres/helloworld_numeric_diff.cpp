//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/helloworld_numeric_diff.h"

namespace xslam {
namespace ceres {

void NumericDiff::RunDemo()
{
    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;

    // Build the problem.
    ::ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // numeric differentiation to obtain the derivative (jacobian).
    ::ceres::CostFunction* cost_function =
            new ::ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ::ceres::CENTRAL, 1, 1>(
                    new NumericDiffCostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);
    // Run the solver!
    ::ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ::ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
}

}
}

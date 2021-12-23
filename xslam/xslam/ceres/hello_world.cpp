//
// Created by quan on 2021/12/6.
//

#include "xslam/ceres/hello_world.h"

namespace xslam {
namespace ceres {

// f(x) = 10 - x
void HelloWorld::RunDemo()
{
    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;

    // Build the problem.
    ::ceres::Problem problem;


    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ::ceres::CostFunction* cost_function =
            new ::ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);

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

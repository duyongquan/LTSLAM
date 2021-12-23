//
// Created by quan on 2021/12/9.
//

#include "xslam/ceres/robot_pose_mle.h"

namespace xslam {
namespace ceres {

void RobotPoseMLE::RunDemo()
{
    std::vector<double> odometry_values;
    std::vector<double> range_readings;

    SimulateRobot(&odometry_values, &range_readings);
    printf("Initial values:\n");
    PrintState(odometry_values, range_readings);
    ::ceres::Problem problem;
    for (int i = 0; i < odometry_values.size(); ++i) {
        // Create and add a DynamicAutoDiffCostFunction for the RangeConstraint from
        // pose i.
        std::vector<double*> parameter_blocks;
        RangeConstraint::RangeCostFunction* range_cost_function =
                RangeConstraint::Create(
                        i, range_readings[i], &odometry_values, &parameter_blocks);

        problem.AddResidualBlock(range_cost_function, NULL, parameter_blocks);
        // Create and add an AutoDiffCostFunction for the OdometryConstraint for pose i.
        problem.AddResidualBlock(OdometryConstraint::Create(odometry_values[i]),
                                 NULL,
                                 &(odometry_values[i]));
    }

    ::ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    ::ceres::Solver::Summary summary;

    printf("Solving...\n");
    Solve(solver_options, &problem, &summary);
    printf("Done.\n");
    std::cout << summary.FullReport() << "\n";
    printf("Final values:\n");
    PrintState(odometry_values, range_readings);
};

void RobotPoseMLE::SimulateRobot(
        std::vector<double>* odometry_values,
        std::vector<double>* range_readings)
{
    const int num_steps = static_cast<int>(ceil(kCorridorLength) / kPoseSeparation);
    // The robot starts out at the origin.
    double robot_location = 0.0;
    for (int i = 0; i < num_steps; ++i)
    {
        const double actual_odometry_value = std::min(kPoseSeparation, kCorridorLength) - robot_location;
        robot_location += actual_odometry_value;
        const double actual_range = kCorridorLength - robot_location;
        const double observed_odometry = RandNormal() * kOdometryStddev + actual_odometry_value;
        const double observed_range = RandNormal() * kRangeStddev + actual_range;

        odometry_values->push_back(observed_odometry);
        range_readings->push_back(observed_range);
    }
}

void RobotPoseMLE::PrintState(
        const std::vector<double>& odometry_readings,
        const std::vector<double>& range_readings)
{
    CHECK_EQ(odometry_readings.size(), range_readings.size());
    double robot_location = 0.0;
    printf("pose: location     odom    range  r.error  o.error\n");
    for (int i = 0; i < odometry_readings.size(); ++i)
    {
        robot_location += odometry_readings[i];
        const double range_error = robot_location + range_readings[i] - kCorridorLength;
        const double odometry_error = kPoseSeparation - odometry_readings[i];
        printf("%4d: %8.3f %8.3f %8.3f %8.3f %8.3f\n",
               static_cast<int>(i),
               robot_location,
               odometry_readings[i],
               range_readings[i],
               range_error,
               odometry_error);
    }
}

} // namespace ceres
} // namespace xslam

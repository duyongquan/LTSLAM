//
// Created by quan on 2021/12/9.
//

#ifndef SLAM_ROBOT_POSE_MLE_H
#define SLAM_ROBOT_POSE_MLE_H

#include <math.h>
#include <cstdio>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"

const double kOdometryStddev = 0.1;
const double kRangeStddev    = 0.01;
const double kCorridorLength = 30.0;
const double kPoseSeparation = 0.5;

namespace xslam {
namespace ceres {

// Return a random number sampled from a uniform distribution in the range
// [0,1].
inline double RandDouble()
{
    double r = static_cast<double>(rand());
    return r / RAND_MAX;
}

// Marsaglia Polar method for generation standard normal (pseudo)
// random numbers http://en.wikipedia.org/wiki/Marsaglia_polar_method
inline double RandNormal()
{
    double x1, x2, w;
    do
    {
        x1 = 2.0 * RandDouble() - 1.0;
        x2 = 2.0 * RandDouble() - 1.0;
        w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 || w == 0.0 );

    w = sqrt((-2.0 * log(w)) / w);
    return x1 * w;
}

struct OdometryConstraint
{
    typedef ::ceres::AutoDiffCostFunction<OdometryConstraint, 1, 1> OdometryCostFunction;
    OdometryConstraint(double odometry_mean, double odometry_stddev)
            : odometry_mean(odometry_mean),
              odometry_stddev(odometry_stddev)
    {}

    template <typename T>
    bool operator()(const T* const odometry, T* residual) const
    {
        *residual = (*odometry - odometry_mean) / odometry_stddev;
        return true;
    }

    static OdometryCostFunction* Create(const double odometry_value)
    {
        return new OdometryCostFunction(new OdometryConstraint(
                odometry_value, kOdometryStddev));
    }

    const double odometry_mean;
    const double odometry_stddev;
};

static constexpr int kStride = 10;

struct RangeConstraint
{
    typedef ::ceres::DynamicAutoDiffCostFunction<RangeConstraint, kStride> RangeCostFunction;
    RangeConstraint(int pose_index,
                    double range_reading,
                    double range_stddev,
                    double corridor_length)
            : pose_index(pose_index),
              range_reading(range_reading),
              range_stddev(range_stddev),
              corridor_length(corridor_length) {}

    template <typename T>
    bool operator()(T const* const* relative_poses, T* residuals) const
    {
        T global_pose(0);
        for (int i = 0; i <= pose_index; ++i)
        {
            global_pose += relative_poses[i][0];
        }
        residuals[0] = (global_pose + range_reading - corridor_length) / range_stddev;
        return true;
    }
    // Factory method to create a CostFunction from a RangeConstraint to
    // conveniently add to a ceres problem.
    static RangeCostFunction* Create(const int pose_index,
                                     const double range_reading,
                                     std::vector<double>* odometry_values,
                                     std::vector<double*>* parameter_blocks)
    {
        RangeConstraint* constraint = new RangeConstraint(pose_index, range_reading,
                kRangeStddev, kCorridorLength);

        RangeCostFunction* cost_function = new RangeCostFunction(constraint);
        // Add all the parameter blocks that affect this constraint.
        parameter_blocks->clear();
        for (int i = 0; i <= pose_index; ++i)
        {
            parameter_blocks->push_back(&((*odometry_values)[i]));
            cost_function->AddParameterBlock(1);
        }
        cost_function->SetNumResiduals(1);
        return (cost_function);
    }

    const int pose_index;
    const double range_reading;
    const double range_stddev;
    const double corridor_length;
};


class RobotPoseMLE
{
public:
    void RunDemo();

private:
    void SimulateRobot(std::vector<double>* odometry_values, std::vector<double>* range_readings);
    void PrintState(const std::vector<double>& odometry_readings, const std::vector<double>& range_readings);
};

} // namespace ceres
} // namespace xslam


#endif //SLAM_ROBOT_POSE_MLE_H

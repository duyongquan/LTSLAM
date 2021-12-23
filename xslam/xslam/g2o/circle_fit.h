//
// Created by quan on 2021/12/21.
//

#ifndef SLAM_CIRCLE_FIT_H
#define SLAM_CIRCLE_FIT_H

#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

namespace xslam {
namespace g2o {

/**
 * A circle located at x,y with radius r
 */
class VertexCircle : public ::g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCircle(){}

    virtual bool read(std::istream& /*is*/);
    virtual bool write(std::ostream& /*os*/) const;

    virtual void setToOriginImpl();
    virtual void oplusImpl(const double* update);
};

/**
 * measurement for a point on the circle
 *
 * Here the measurement is the point which is on the circle.
 * The error function computes the distance of the point to
 * the center minus the radius of the circle.
 */
class EdgePointOnCircle : public ::g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePointOnCircle() {}

    virtual bool read(std::istream & /*is*/);
    virtual bool write(std::ostream & /*os*/) const;

    void computeError();
};

class CircleFit
{
public:
    // some handy typedefs
    using MyBlockSolver = ::g2o::BlockSolver<::g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>  ;
    using MyLinearSolver = ::g2o::LinearSolverCSparse<MyBlockSolver::PoseMatrixType>;

    void RunDemo();

private:
    double ErrorOfSolution(int numPoints, Eigen::Vector2d* points, const Eigen::Vector3d& circle);
};

} // namespace g2o
} // namespace xslam

#endif //SLAM_CIRCLE_FIT_H

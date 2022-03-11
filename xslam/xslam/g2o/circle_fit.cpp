//
// Created by quan on 2021/12/21.
//

#include "xslam/g2o/circle_fit.h"


#if 0
namespace xslam {
namespace g2o {

bool VertexCircle::read(std::istream& /*is*/)
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool VertexCircle::write(std::ostream& /*os*/) const
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

void VertexCircle::setToOriginImpl()
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
}

void VertexCircle::oplusImpl(const double* update)
{
    Eigen::Vector3d::ConstMapType v(update);
    _estimate += v;
}


bool EdgePointOnCircle::read(std::istream& /*is*/)
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool EdgePointOnCircle::write(std::ostream& /*os*/) const
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

void EdgePointOnCircle::computeError()
{
    const VertexCircle* circle = static_cast<const VertexCircle*>(vertex(0));

    const Eigen::Vector2d& center = circle->estimate().head<2>();
    const double& radius = circle->estimate()(2);

    _error(0) = (measurement() - center).norm() - radius;
}


void CircleFit::RunDemo()
{
    int numPoints = 100;
    int maxIterations = 10;
    bool verbose = true;

    // --1 generate random data
    Eigen::Vector2d center(4.0, 2.0);
    double radius = 2.0;
    Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];

    ::g2o::Sampler::seedRand();
    for (int i = 0; i < numPoints; ++i) {
        double r = ::g2o::Sampler::gaussRand(radius, 0.05);
        double angle = ::g2o::Sampler::uniformRand(0.0, 2.0 * M_PI);
        points[i].x() = center.x() + r * cos(angle);
        points[i].y() = center.y() + r * sin(angle);
    }

    // --2 setup the solver
    ::g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    ::g2o::OptimizationAlgorithmLevenberg* solver = new ::g2o::OptimizationAlgorithmLevenberg(
            ::g2o::make_unique<MyBlockSolver>(::g2o::make_unique<MyLinearSolver>()));
    optimizer.setAlgorithm(solver);

    // --3 build the optimization problem given the points
    // 3.1. add the circle vertex
    VertexCircle* circle = new VertexCircle();
    circle->setId(0);
    circle->setEstimate(Eigen::Vector3d(3.0, 3.0, 3.0)); // some initial value for the circle
    optimizer.addVertex(circle);
    // 3.2. add the points we measured
    for (int i = 0; i < numPoints; ++i) {
        EdgePointOnCircle* e = new EdgePointOnCircle;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->setVertex(0, circle);
        e->setMeasurement(points[i]);
        optimizer.addEdge(e);
    }

    // --4 perform the optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(verbose);
    optimizer.optimize(maxIterations);

    if (verbose) {
        std::cout << std::endl;
    }


    // --5 print out the result
    std::cout << "Iterative least squares solution" << std::endl;
    std::cout << "center of the circle " << circle->estimate().head<2>().transpose() << std::endl;
    std::cout << "radius of the cirlce " << circle->estimate()(2) << std::endl;
    std::cout << "error " << ErrorOfSolution(numPoints, points, circle->estimate()) << std::endl;
    std::cout << std::endl;

    // solve by linear least squares
    // Let (a, b) be the center of the circle and r the radius of the circle.
    // For a point (x, y) on the circle we have:
    // (x - a)^2 + (y - b)^2 = r^2
    // This leads to
    // (-2x -2y 1)^T * (a b c) = -x^2 - y^2   (1)
    // where c = a^2 + b^2 - r^2.
    // Since we have a bunch of points, we accumulate Eqn (1) in a matrix and
    // compute the normal equation to obtain a solution for (a b c).
    // Afterwards the radius r is recovered.
    Eigen::MatrixXd A(numPoints, 3);
    Eigen::VectorXd b(numPoints);
    for (int i = 0; i < numPoints; ++i) {
        A(i, 0) = -2*points[i].x();
        A(i, 1) = -2*points[i].y();
        A(i, 2) = 1;
        b(i) = -pow(points[i].x(), 2) - pow(points[i].y(), 2);
    }
    Eigen::Vector3d solution = (A.transpose()*A).ldlt().solve(A.transpose() * b);
    // calculate the radius of the circle given the solution so far
    solution(2) = sqrt(pow(solution(0), 2) + pow(solution(1), 2) - solution(2));
    std::cout << "Linear least squares solution" << std::endl;
    std::cout << "center of the circle " << solution.head<2>().transpose() << std::endl;
    std::cout << "radius of the cirlce " << solution(2) << std::endl;
    std::cout << "error " << ErrorOfSolution(numPoints, points, solution) << std::endl;

    // clean up
    delete[] points;

}

double CircleFit::ErrorOfSolution(int numPoints, Eigen::Vector2d* points, const Eigen::Vector3d& circle)
{
    Eigen::Vector2d center = circle.head<2>();
    double radius = circle(2);
    double error = 0.;
    for (int i = 0; i < numPoints; ++i) {
        double d = (points[i] - center).norm() - radius;
        error += d*d;
    }
    return error;
}

} // namespace g2o
} // namespace xslam

#endif 
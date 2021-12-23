//
// Created by quan on 2021/12/9.
//

#ifndef SLAM_TYPES_H
#define SLAM_TYPES_H

#include "xslam/ceres/normalize_angle.h"

#include <string>
#include <fstream>

#include "Eigen/Core"

namespace xslam {
namespace ceres {
namespace example {

/****************************************** 2D ******************************************/
// The state for each vertex in the pose graph.
struct Pose2d
{
    double x;
    double y;
    double yaw_radians;

    // The name of the data type in the g2o file format.
    static std::string name() {
        return "VERTEX_SE2";
    }
};

std::istream& operator>>(std::istream& input, Pose2d& pose);

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint2d
{
    int id_begin;
    int id_end;

    double x;
    double y;
    double yaw_radians;

    // The inverse of the covariance matrix for the measurement. The order of the
    // entries are x, y, and yaw.
    Eigen::Matrix3d information;

    // The name of the data type in the g2o file format.
    static std::string name() {
        return "EDGE_SE2";
    }
};

std::istream& operator>>(std::istream& input, Constraint2d& constraint);

/****************************************** 3D ******************************************/
struct Pose3d
{
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    // The name of the data type in the g2o file format.
    static std::string name() {
        return "VERTEX_SE3:QUAT";
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::istream& operator>>(std::istream& input, Pose3d& pose);

typedef std::map<int, Pose3d, std::less<int>,
        Eigen::aligned_allocator<std::pair<const int, Pose3d> > >
        MapOfPoses;

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint3d {
    int id_begin;
    int id_end;

    // The transformation that represents the pose of the end frame E w.r.t. the
    // begin frame B. In other words, it transforms a vector in the E frame to
    // the B frame.
    Pose3d t_be;

    // The inverse of the covariance matrix for the measurement. The order of the
    // entries are x, y, z, delta orientation.
    Eigen::Matrix<double, 6, 6> information;

    // The name of the data type in the g2o file format.
    static std::string name() {
        return "EDGE_SE3:QUAT";
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::istream& operator>>(std::istream& input, Constraint3d& constraint);

typedef std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d> > VectorOfConstraints;

} // namespace example
} // namespace ceres
} // namespace xslam

#endif //SLAM_TYPES_H

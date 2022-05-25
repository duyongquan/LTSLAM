#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_POSE_ESTIMATION_3D2D_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_POSE_ESTIMATION_3D2D_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>

#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace xslam {
namespace opencv {
namespace geometry_transform {

// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override 
    {
        _estimate = Sophus::SE3d();
    }

    // left multiplication on SE3
    virtual void oplusImpl(const double *update) override 
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override {}
    virtual bool write(std::ostream &out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

    virtual void computeError() override 
    {
            const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override 
    {
        const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pos3d;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double cx = _K(0, 2);
        double cy = _K(1, 2);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Z2 = Z * Z;
        _jacobianOplusXi
            << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
            0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    }

    virtual bool read(std::istream &in) override {}
    virtual bool write(std::ostream &out) const override {}

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};

class PoseEstimation3D2D
{
public:
    // BA by g2o
    using VecVector2d = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
    using VecVector3d = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

    void RunDemo(const std::string& iamge_1, const std::string& iamge_depth_1, 
                 const std::string& iamge_2, const std::string& iamge_depth_2);

private:
    void FindFeatureMatches(
            const cv::Mat &img_1, const cv::Mat &img_2,
            std::vector<cv::KeyPoint> &keypoints_1,
            std::vector<cv::KeyPoint> &keypoints_2,
            std::vector<cv::DMatch> &matches);

    void BundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K, 
        Sophus::SE3d &pose);

    // BA by gauss-newton
    void BundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose);

    // 像素坐标转相机归一化坐标
    cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K);
};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_POSE_ESTIMATION_3D2D_H

#ifndef XSLAM_OPENCV_DIRECT_METHOD_H
#define XSLAM_OPENCV_DIRECT_METHOD_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>

// useful typedefs
using Matrix6d  = Eigen::Matrix<double, 6, 6>;
using Matrix26d = Eigen::Matrix<double, 2, 6>;
using Vector6d  = Eigen::Matrix<double, 6, 1>;
using VecVector2d = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

namespace xslam {
namespace opencv {

struct Camera
{
    // Camera intrinsics
    double fx = 718.856;
    double fy = 718.856;
    double cx = 607.1928;
    double cy = 185.2157;

    // baseline
    double baseline = 0.573;
};

// bilinear interpolation
float GetPixelValue(const cv::Mat &img, float x, float y);

class DirectMetod
{
public:
    void RunDemo(const std::string& filename);

private:

    /**
     * pose estimation using direct method
     * @param img1
     * @param img2
     * @param px_ref
     * @param depth_ref
     * @param T21
     */
    void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const std::vector<double> depth_ref,
        Sophus::SE3d &T21
    );

    /**
     * pose estimation using direct method
     * @param img1
     * @param img2
     * @param px_ref
     * @param depth_ref
     * @param T21
     */
    void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const std::vector<double> depth_ref,
        Sophus::SE3d &T21
    );

    struct JacobianAccumulator 
    {
        JacobianAccumulator(
        const cv::Mat &img1_,
        const cv::Mat &img2_,
        const VecVector2d &px_ref_,
        const std::vector<double> depth_ref_,
        Sophus::SE3d &T21_) :
        img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_) 
        {
            projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
        }

        // accumulate jacobians in a range
        void accumulate_jacobian(const cv::Range &range);

        // get hessian matrix
        Matrix6d hessian() const { return H; }

        // get bias
        Vector6d bias() const { return b; }

        // get total cost
        double cost_func() const { return cost; }

        // get projected points
        VecVector2d projected_points() const { return projection; }

        /// reset h, b, cost to zero
        void reset() 
        {
            H = Matrix6d::Zero();
            b = Vector6d::Zero();
            cost = 0;
        }
        
        const cv::Mat &img1;
        const cv::Mat &img2;
        const VecVector2d &px_ref;
        const std::vector<double> depth_ref;
        Sophus::SE3d &T21;
        VecVector2d projection; // projected points

        std::mutex hessian_mutex;
        Matrix6d H = Matrix6d::Zero();
        Vector6d b = Vector6d::Zero();
        double cost = 0;
    };

    
};

} // namespace opencv
} // namespace xslam

#endif //XSLAM_OPENCV_DIRECT_METHOD_H
#ifndef XSLAM_VINS_CAMERA_PINHOLE_CAMERA_H
#define XSLAM_VINS_CAMERA_PINHOLE_CAMERA_H

#include "xslam/vins/camera/camera_interface.h"
#include "xslam/vins/camera/camera_options.h"
#include "xslam/vins/camera/proto/pinhole_camera_options.pb.h"
#include <string>

#include "opencv2/core/core.hpp"
#include "ceres/rotation.h"


namespace xslam {
namespace vins {
namespace camera {

class PinholeCamera : public CameraInterface
{
public: 
    PinholeCamera(proto::PinholeCameraOptions& options);
    virtual ~PinholeCamera();

    PinholeCamera(const PinholeCamera& other) = delete;
    PinholeCamera& operator=(const PinholeCamera& ohter) = delete;

    // Lift points from the image plane to the sphere
    // `pixel` image coordinates
    // `Pose` coordinates of the point on the sphere
    virtual void LiftSphere(const Eigen::Vector2d& pixel, Eigen::Vector3d& Pose) const override;

    // Lift points from the image plane to the projective space
    // Lifts a point from the image plane to its projective ray
    // `pixel` image coordinates
    // `Pose` coordinates of the projective ray
    // 2D --> 3D
    virtual void LiftProjective(const Eigen::Vector2d& pixel, Eigen::Vector3d& Pose) const override;

    // Projects 3D points to the image plane (Pi function)
    // Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
    // `Pose` 3D point coordinates
    // `pixel` return value, contains the image point coordinates
    // 3D --> 2D
    virtual void SpaceToPlane(const Eigen::Vector3d& Pose, Eigen::Vector2d& pixel) const override;

    // Projects an undistorted 2D point p_u to the image plane
    // `pixel_undistoriton` 2D point coordinates
    // rreturn image point coordinates
    virtual void UndistToPlane(const Eigen::Vector2d& pixel_undistoriton, Eigen::Vector2d& pixel) const override;

    virtual void DebugString() const override;

    void EstimateIntrinsics(const cv::Size& board_size,
                            const std::vector< std::vector<cv::Point3f> >& object_points,
                            const std::vector< std::vector<cv::Point2f> >& image_points);

    virtual ModelType model_type(void) const override;
    virtual const std::string& camera_name(void) const override;
    virtual int image_width(void) const override;
    virtual int image_height(void) const override;

    
    template <typename T>
    static void SpaceToPlane(const T* const params,
                             const T* const q, const T* const t,
                             const Eigen::Matrix<T, 3, 1>& P,
                             Eigen::Matrix<T, 2, 1>& p);

    // Apply distortion to input point (from the normalised plane)
    // `p_u` undistorted coordinates of point on the normalised plane
    // return to obtain the distorted point: p_d = p_u + d_u
    void Distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;

    // Apply distortion to input point (from the normalised plane) and calculate Jacobian
    // `p_u` undistorted coordinates of point on the normalised plane
    // return to obtain the distorted point: p_d = p_u + d_u
    void Distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u, Eigen::Matrix2d& J) const;

    void InitializeUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat InitializeUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                          float fx = -1.0f, float fy = -1.0f,
                                          cv::Size image_size = cv::Size(0, 0),
                                          float cx = -1.0f, float cy = -1.0f,
                                          cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

private:
    proto::PinholeCameraOptions options_;
    double inverse_k11_;
    double inverse_k13_;
    double inverse_k22_;
    double inverse_k23_;
    bool no_distortion_;
};

using PinholeCameraPtr = std::shared_ptr<PinholeCamera>;
using PinholeCameraConstPtr = std::shared_ptr<const PinholeCamera>;

template <typename T>
void PinholeCamera::SpaceToPlane(
    const T* const params,
    const T* const q, const T* const t,
    const Eigen::Matrix<T, 3, 1>& P,
    Eigen::Matrix<T, 2, 1>& p)
{
    T P_w[3];
    P_w[0] = T(P(0));
    P_w[1] = T(P(1));
    P_w[2] = T(P(2));

    // Convert quaternion from Eigen convention (x, y, z, w)
    // to Ceres convention (w, x, y, z)
    T q_ceres[4] = {q[3], q[0], q[1], q[2]};

    T P_c[3];
    ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

    P_c[0] += t[0];
    P_c[1] += t[1];
    P_c[2] += t[2];

    // project 3D object point to the image plane
    T k1 = params[0];
    T k2 = params[1];
    T p1 = params[2];
    T p2 = params[3];
    T fx = params[4];
    T fy = params[5];
    T alpha = T(0); //cameraParams.alpha();
    T cx = params[6];
    T cy = params[7];

    // Transform to model plane
    T u = P_c[0] / P_c[2];
    T v = P_c[1] / P_c[2];

    T rho_sqr = u * u + v * v;
    T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
    T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u);
    T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

    u = L * u + du;
    v = L * v + dv;
    p(0) = fx * (u + alpha * v) + cx;
    p(1) = fy * v + cy;
}

}  // namespace camera
}  // namespace vins
}  // namespace xslam

#endif // XSLAM_VINS_CAMERA_PINHOLE_CAMERA_H
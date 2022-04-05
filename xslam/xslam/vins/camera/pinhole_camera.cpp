#include "xslam/vins/camera/pinhole_camera.h"

#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace camera {

PinholeCamera::PinholeCamera(proto::PinholeCameraOptions& options)
    : options_(options)
{

}

void PinholeCamera::LiftSphere(const Eigen::Vector2d& pixel, Eigen::Vector3d& Pose) const
{
    LiftProjective(pixel, Pose);
    Pose.normalize();
}

void PinholeCamera::LiftProjective(const Eigen::Vector2d& pixel, Eigen::Vector3d& Pose) const
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;

    // Lift points to normalised plane
    mx_d = inverse_k11_ * pixel(0) + inverse_k13_;
    my_d = inverse_k22_ * pixel(1) + inverse_k23_;

    if (no_distortion_)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
        if (0)
        {
            double k1 = options_.paramters().k1();
            double k2 = options_.paramters().k2();
            double p1 = options_.paramters().p1();
            double p2 = options_.paramters().p2();

            // Apply inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }
        else
        {
            // Recursive distortion model
            int n = 8;
            Eigen::Vector2d d_u;
            Distortion(Eigen::Vector2d(mx_d, my_d), d_u);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            for (int i = 1; i < n; ++i)
            {
                Distortion(Eigen::Vector2d(mx_u, my_u), d_u);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }
        }
    }

    // Obtain a projective ray
    Pose << mx_u, my_u, 1.0;
}

void PinholeCamera::SpaceToPlane(const Eigen::Vector3d& Pose, Eigen::Vector2d& pixel) const
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    p_u << Pose(0) / Pose(2), Pose(1) / Pose(2);

    if (no_distortion_)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        Distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    pixel << options_.paramters().fx() * p_d(0) + options_.paramters().cx(),
             options_.paramters().fy() * p_d(1) + options_.paramters().cy();
}

void PinholeCamera::UndistToPlane(const Eigen::Vector2d& pixel_undistoriton, Eigen::Vector2d& pixel) const
{
    Eigen::Vector2d pixel_distoriton;

    if (no_distortion_)
    {
        pixel_distoriton = pixel_undistoriton;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        Distortion(pixel_undistoriton, d_u);
        pixel_distoriton = pixel_undistoriton + d_u;
    }

    // Apply generalised projection matrix
    pixel << options_.paramters().fx() * pixel_distoriton(0) + options_.paramters().cx(),
             options_.paramters().fy() * pixel_distoriton(1) + options_.paramters().cy();
}

void PinholeCamera::DebugString() const
{
    LOG(INFO) << "Camera Parameters:";
    LOG(INFO) << "   model_type " << "PINHOLE";;
    LOG(INFO) << "   camera_name " << camera_name();
    LOG(INFO) << "   image_width " << image_width();
    LOG(INFO) << "   image_height " << image_height();

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    LOG(INFO) << "Distortion Parameters";
    LOG(INFO) << "            k1 " << options_.paramters().k1();
    LOG(INFO) << "            k2 " << options_.paramters().k2();
    LOG(INFO) << "            p1 " << options_.paramters().p1();
    LOG(INFO) << "            p2 " << options_.paramters().p2();

    // projection: fx, fy, cx, cy
    LOG(INFO) << "Projection Parameters";
    LOG(INFO) << "            fx " << options_.paramters().fx();
    LOG(INFO) << "            fy " << options_.paramters().fy();
    LOG(INFO) << "            cx " << options_.paramters().cx();
    LOG(INFO) << "            cy " << options_.paramters().cy();
}

void PinholeCamera::EstimateIntrinsics(
    const cv::Size& board_size,
    const std::vector< std::vector<cv::Point3f> >& object_points,
    const std::vector< std::vector<cv::Point2f> >& image_points)
{
    // Z. Zhang, A Flexible New Technique for Camera Calibration, PAMI 2000
    PinholeOptions params;
    params.k1 = 0.0;
    params.k2 = 0.0;
    params.p1 = 0.0;
    params.p2 = 0.0;

    params.cx = image_width() / 2.0;
    params.cy = image_height() / 2.0;
   

    size_t nImages = image_points.size();

    cv::Mat A(nImages * 2, 2, CV_64F);
    cv::Mat b(nImages * 2, 1, CV_64F);

    for (size_t i = 0; i < nImages; ++i)
    {
        const std::vector<cv::Point3f>& oPoints = object_points.at(i);

        std::vector<cv::Point2f> M(oPoints.size());
        for (size_t j = 0; j < M.size(); ++j)
        {
            M.at(j) = cv::Point2f(oPoints.at(j).x, oPoints.at(j).y);
        }

        cv::Mat H = cv::findHomography(M, image_points.at(i));

        H.at<double>(0,0) -= H.at<double>(2,0) * params.cx;
        H.at<double>(0,1) -= H.at<double>(2,1) * params.cx;
        H.at<double>(0,2) -= H.at<double>(2,2) * params.cx;
        H.at<double>(1,0) -= H.at<double>(2,0) * params.cy;
        H.at<double>(1,1) -= H.at<double>(2,1) * params.cy;
        H.at<double>(1,2) -= H.at<double>(2,2) * params.cy;

        double h[3], v[3], d1[3], d2[3];
        double n[4] = {0,0,0,0};

        for (int j = 0; j < 3; ++j)
        {
            double t0 = H.at<double>(j,0);
            double t1 = H.at<double>(j,1);
            h[j] = t0; v[j] = t1;
            d1[j] = (t0 + t1) * 0.5;
            d2[j] = (t0 - t1) * 0.5;
            n[0] += t0 * t0; n[1] += t1 * t1;
            n[2] += d1[j] * d1[j]; n[3] += d2[j] * d2[j];
        }

        for (int j = 0; j < 4; ++j)
        {
            n[j] = 1.0 / sqrt(n[j]);
        }

        for (int j = 0; j < 3; ++j)
        {
            h[j] *= n[0]; v[j] *= n[1];
            d1[j] *= n[2]; d2[j] *= n[3];
        }

        A.at<double>(i * 2, 0) = h[0] * v[0];
        A.at<double>(i * 2, 1) = h[1] * v[1];
        A.at<double>(i * 2 + 1, 0) = d1[0] * d2[0];
        A.at<double>(i * 2 + 1, 1) = d1[1] * d2[1];
        b.at<double>(i * 2, 0) = -h[2] * v[2];
        b.at<double>(i * 2 + 1, 0) = -d1[2] * d2[2];
    }

    cv::Mat f(2, 1, CV_64F);
    cv::solve(A, b, f, cv::DECOMP_NORMAL | cv::DECOMP_LU);

    params.fx = sqrt(fabs(1.0 / f.at<double>(0)));
    params.fy = sqrt(fabs(1.0 / f.at<double>(1)));
    ToProto(params);
}

ModelType PinholeCamera::model_type(void) const
{
    if (options_.info().model_type() == "PINHOLE") {
        return ModelType::kPinhole;
    }
    return ModelType::kUnknown;
}

const std::string& PinholeCamera::camera_name(void) const
{
    return options_.info().camera_name();
}

int PinholeCamera::image_width(void) const
{
    return options_.info().image_width();
}

int PinholeCamera::image_height(void) const
{
    return options_.info().image_height();
}

void PinholeCamera::Distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
{
    double k1 = options_.paramters().k1();
    double k2 = options_.paramters().k2();
    double p1 = options_.paramters().p1();
    double p2 = options_.paramters().p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void PinholeCamera::Distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u, Eigen::Matrix2d& J) const
{
    double k1 = options_.paramters().k1();
    double k2 = options_.paramters().k2();
    double p1 = options_.paramters().p1();
    double p2 = options_.paramters().p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);

    double dxdmx = 1.0 + rad_dist_u + k1 * 2.0 * mx2_u + k2 * rho2_u * 4.0 * mx2_u + 2.0 * p1 * p_u(1) + 6.0 * p2 * p_u(0);
    double dydmx = k1 * 2.0 * p_u(0) * p_u(1) + k2 * 4.0 * rho2_u * p_u(0) * p_u(1) + p1 * 2.0 * p_u(0) + 2.0 * p2 * p_u(1);
    double dxdmy = dydmx;
    double dydmy = 1.0 + rad_dist_u + k1 * 2.0 * my2_u + k2 * rho2_u * 4.0 * my2_u + 6.0 * p1 * p_u(1) + 2.0 * p2 * p_u(0);

    J << dxdmx, dxdmy,
         dydmx, dydmy;
}

void PinholeCamera::InitializeUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale) const
{
    cv::Size image_size(image_width(),image_height());

    cv::Mat map_x = cv::Mat::zeros(image_size, CV_32F);
    cv::Mat map_y = cv::Mat::zeros(image_size, CV_32F);

    for (int v = 0; v < image_size.height; ++v)
    {
        for (int u = 0; u < image_size.width; ++u)
        {
            double mx_u = inverse_k11_ / fScale * u + inverse_k13_ / fScale;
            double my_u = inverse_k22_ / fScale * v + inverse_k23_ / fScale;

            Eigen::Vector3d P;
            P << mx_u, my_u, 1.0;

            Eigen::Vector2d p;
            SpaceToPlane(P, p);

            map_x.at<float>(v,u) = p(0);
            map_y.at<float>(v,u) = p(1);
        }
    }
    cv::convertMaps(map_x, map_y, map1, map2, CV_32FC1, false);
}

cv::Mat PinholeCamera::InitializeUndistortRectifyMap(
    cv::Mat& map1, cv::Mat& map2, float fx, float fy,
    cv::Size image_size, float cx , float cy, cv::Mat rmat) const
{
    if (image_size == cv::Size(0, 0))
    {
        image_size = cv::Size(image_width(), image_height());
    }

    cv::Mat map_x = cv::Mat::zeros(image_size.height, image_size.width, CV_32F);
    cv::Mat map_y = cv::Mat::zeros(image_size.height, image_size.width, CV_32F);

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen(rmat, R);
    R_inv = R.inverse();

    // assume no skew
    Eigen::Matrix3f K_rect;

    if (cx == -1.0f || cy == -1.0f)
    {
        K_rect << fx, 0, image_size.width / 2,
                  0, fy, image_size.height / 2,
                  0, 0, 1;
    }
    else
    {
        K_rect << fx, 0, cx,
                  0, fy, cy,
                  0, 0, 1;
    }

    if (fx == -1.0f || fy == -1.0f)
    {
        K_rect(0,0) = options_.paramters().fx();
        K_rect(1,1) = options_.paramters().fy();
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse();

    for (int v = 0; v < image_size.height; ++v)
    {
        for (int u = 0; u < image_size.width; ++u)
        {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            Eigen::Vector2d p;
            SpaceToPlane(uo.cast<double>(), p);

            map_x.at<float>(v,u) = p(0);
            map_y.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(map_x, map_y, map1, map2, CV_32FC1, false);

    cv::Mat K_rect_cv;
    cv::eigen2cv(K_rect, K_rect_cv);
    return K_rect_cv;
}

}  // namespace camera
}  // namespace vins
}  // namespace xslam
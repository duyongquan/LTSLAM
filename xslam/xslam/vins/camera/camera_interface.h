#ifndef XSLAM_VINS_CAMERA_CAMERA_INTERFACE_H
#define XSLAM_VINS_CAMERA_CAMERA_INTERFACE_H

#include "xslam/vins/camera/camera_options.h"

#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace xslam {
namespace vins {
namespace camera {

class CameraInterface 
{
public:
    CameraInterface() {};
    virtual ~CameraInterface() {}

    CameraInterface(const CameraInterface& other) = delete;
    CameraInterface& operator=(const CameraInterface& ohter) = delete;

    // Lift points from the image plane to the sphere
    virtual void LiftSphere(const Eigen::Vector2d& pixel, Eigen::Vector3d& Pose) const = 0;

    // Lift points from the image plane to the projective space
    // 2D --> 3D
    virtual void LiftProjective(const Eigen::Vector2d& pixel, Eigen::Vector3d& Pose) const = 0;

    // Projects 3D points to the image plane (Pi function)
    // 3D --> 2D
    virtual void SpaceToPlane(const Eigen::Vector3d& Pose, Eigen::Vector2d& pixel) const = 0;

    virtual void UndistToPlane(const Eigen::Vector2d& pixel_undistoriton, Eigen::Vector2d& pixel) const = 0;

    virtual void DebugString() const = 0;

    // Calculates the reprojection distance between points
    // `Pose1` first 3D point coordinates
    // `Pose2` second 3D point coordinates
    // return euclidean distance in the plane
    // double ReprojectionDistance(const Eigen::Vector3d& Pose1, const Eigen::Vector3d& Pose2) const;

    // double ReprojectionError(const std::vector<std::vector<cv::Point3f>>& object_points,
    //                          const std::vector<std::vector<cv::Point2f>>& image_points,
    //                          const std::vector<cv::Mat>& R,
    //                          const std::vector<cv::Mat>& t,
    //                          cv::OutputArray perViewErrors = cv::noArray());

    // double ReprojectionError(const Eigen::Vector3d& Pose,
    //                          const Eigen::Quaterniond& camera_quaternion,
    //                          const Eigen::Vector3d& camera_t,
    //                          const Eigen::Vector2d& observed_p) const;

    // void ProjectPoints(const std::vector<cv::Point3f>& object_points,
    //                    const cv::Mat& R,
    //                    const cv::Mat& t,
    //                    std::vector<cv::Point2f>& image_points) const;

    // options 
    virtual ModelType model_type(void) const = 0;
    virtual const std::string& camera_name(void) const = 0;
    virtual int image_width(void) const = 0;
    virtual int image_height(void) const = 0;

protected:
    cv::Mat mask_;
};

using CameraPtr = std::shared_ptr<CameraInterface>;
using CameraConstPtr = std::shared_ptr<const CameraInterface>;

}  // namespace camera
}  // namespace vins
}  // namespace xslam

#endif // XSLAM_VINS_CAMERA_CAMERA_INTERFACE_H
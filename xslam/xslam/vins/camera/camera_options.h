#ifndef XSLAM_VINS_CAMERA_CAMERA_OPTIONS_H
#define XSLAM_VINS_CAMERA_CAMERA_OPTIONS_H

#include "xslam/vins/camera/proto/pinhole_camera_options.pb.h"

#include <string>

namespace xslam {
namespace vins {
namespace camera {

enum class ModelType
{
    kUnknown,
    kMei,
    kPinhole,
    kScaramuzza,
    kKannalaBrandt
};

struct CameraOptions
{
    ModelType model_type;
    int intrinsics_opitons;
    std::string camera_name;
    int image_width;
    int image_height;
};

struct PinholeOptions
{
    double k1;
    double k2;
    double p1;
    double p2;
    double fx;
    double fy;
    double cx;
    double cy;
};

struct PinholeCameraOpionts
{
    CameraOptions info;
    PinholeOptions parameters;
};

CameraOptions FromProto(const proto::CameraOptions& proto);
proto::CameraOptions ToProto(const CameraOptions& camera_options);

PinholeOptions FromProto(const proto::PinholeOptions& proto);
proto::PinholeOptions ToProto(const PinholeOptions& pinhole_options);

PinholeCameraOpionts FromProto(const proto::PinholeCameraOptions& proto);
proto::PinholeCameraOptions ToProto(const PinholeCameraOpionts& pinhole_camera_options);


}  // namespace camera
}  // namespace vins
}  // namespace xslam





#endif // XSLAM_VINS_CAMERA_CAMERA_OPTIONS_H



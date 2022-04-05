#include "xslam/vins/camera/camera_options.h"

namespace xslam {
namespace vins {
namespace camera {

constexpr char kPinholeModelType[] = "PINHOLE";

CameraOptions FromProto(const proto::CameraOptions& proto)
{
    ModelType model_type;
    if (proto.model_type() == kPinholeModelType) {
        model_type = ModelType::kPinhole;
    }

    return CameraOptions{
        model_type,
        proto.intrinsics(),
        proto.camera_name(),
        proto.image_width(),
        proto.image_height()
    };
}

proto::CameraOptions ToProto(const CameraOptions& camera_options)
{
    proto::CameraOptions proto;
    std::string model_type = "unknown";
    if (camera_options.model_type == ModelType::kPinhole) {
        model_type = kPinholeModelType;
    }

    proto.set_model_type(model_type);
    proto.set_intrinsics(camera_options.intrinsics_opitons);
    proto.set_camera_name(camera_options.camera_name);
    proto.set_image_width(camera_options.image_width);
    proto.set_image_height(camera_options.image_height);
    return proto;
}

PinholeOptions FromProto(const proto::PinholeOptions& proto)
{
    return PinholeOptions{
        proto.k1(),
        proto.k2(),
        proto.p1(),
        proto.p2(),
        proto.fx(),
        proto.fy(),
        proto.cx(),
        proto.cy(),
    };
}

proto::PinholeOptions ToProto(const PinholeOptions& pinhole_options)
{
    proto::PinholeOptions proto;
    proto.set_k1(pinhole_options.k1);
    proto.set_k2(pinhole_options.k2);
    proto.set_p1(pinhole_options.p1);
    proto.set_p2(pinhole_options.p2);
    proto.set_fx(pinhole_options.fx);
    proto.set_fy(pinhole_options.fy);
    proto.set_cx(pinhole_options.cx);
    proto.set_cy(pinhole_options.cy);
    return proto;
}

PinholeCameraOpionts FromProto(const proto::PinholeCameraOptions& proto)
{
    return {
        FromProto(proto.info()),
        FromProto(proto.paramters())
    };
}

proto::PinholeCameraOptions ToProto(const PinholeCameraOpionts& pinhole_camera_options)
{
    proto::PinholeCameraOptions proto;
    *proto.mutable_info() = ToProto(pinhole_camera_options.info);
    *proto.mutable_paramters() = ToProto(pinhole_camera_options.parameters);
    return proto;
}

}  // namespace camera
}  // namespace vins
}  // namespace xslam

 
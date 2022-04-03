#ifndef XSLAM_VINS_SENSOR_IMU_DATA_H
#define XSLAM_VINS_SENSOR_IMU_DATA_H

#include "Eigen/Core"
#include "xslam/vins/common/time.h"

namespace xslam {
namespace vins {
namespace sensor {

struct ImuData 
{
    common::Time time;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

}  // namespace sensor
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_SENSOR_IMU_DATA_H
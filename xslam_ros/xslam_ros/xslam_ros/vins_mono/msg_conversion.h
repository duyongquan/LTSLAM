#ifndef XSLAM_ROS_VINS_MONO_MSG_CONVERSION_H
#define XSLAM_ROS_VINS_MONO_MSG_CONVERSION_H

#include "xslam/vins/transform/rigid_transform.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/sensor/image_data.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace xslam_ros {
namespace vins_mono {

::xslam::vins::transform::Rigid3d ToRigid3d(
    const geometry_msgs::TransformStamped& transform);

::xslam::vins::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);

} // namespace vins_mono 
} // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_MSG_CONVERSION_H

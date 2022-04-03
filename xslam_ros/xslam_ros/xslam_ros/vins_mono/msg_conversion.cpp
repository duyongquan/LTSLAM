#include "xslam_ros/vins_mono/msg_conversion.h"


namespace xslam_ros {
namespace vins_mono {

using ::xslam::vins::transform::Rigid3d;

::xslam::vins::transform::Rigid3d ToRigid3d(
    const geometry_msgs::TransformStamped& transform)
{
      return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

::xslam::vins::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose)
{
    return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

} // namespace vins_mono 
} // namespace xslam_ros
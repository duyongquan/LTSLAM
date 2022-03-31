#include "xslam_ros/vins_mono/msg_conversion.h"


namespace xslam_ros {
namespace vins_mono {

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

} // namespace vins_mono 
} // namespace xslam_ros
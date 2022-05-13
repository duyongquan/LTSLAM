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

sensor_msgs::PointCloud ToPointCloud(const xslam::vins::common::messages::PointCloud& point_cloud)
{
  sensor_msgs::PointCloud feature_points;
  // feature_points->header = img_msg->header;
  feature_points.header.frame_id = "world";
  feature_points.channels = point_cloud.point_cloud;
  return feature_points;
}

} // namespace vins_mono 
} // namespace xslam_ros
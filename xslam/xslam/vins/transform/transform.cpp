#include "xslam/vins/transform/transform.h"

namespace xslam {
namespace vins {
namespace transform {

Rigid2d ToRigid2(const proto::Rigid2d& transform) {
  return Rigid2d({transform.translation().x(), transform.translation().y()},
                 transform.rotation());
}

Eigen::Vector2d ToEigen(const proto::Vector2d& vector) {
  return Eigen::Vector2d(vector.x(), vector.y());
}

Eigen::Vector3f ToEigen(const proto::Vector3f& vector) {
  return Eigen::Vector3f(vector.x(), vector.y(), vector.z());
}

Eigen::Vector4f ToEigen(const proto::Vector4f& vector) {
  return Eigen::Vector4f(vector.x(), vector.y(), vector.z(), vector.t());
}

Eigen::Vector3d ToEigen(const proto::Vector3d& vector) {
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

Eigen::Quaterniond ToEigen(const proto::Quaterniond& quaternion) {
  return Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(),
                            quaternion.z());
}

proto::Rigid2d ToProto(const transform::Rigid2d& transform) {
  proto::Rigid2d proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}

proto::Rigid2f ToProto(const transform::Rigid2f& transform) {
  proto::Rigid2f proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}

proto::Rigid3d ToProto(const transform::Rigid3d& rigid) {
  proto::Rigid3d proto;
  *proto.mutable_translation() = ToProto(rigid.translation());
  *proto.mutable_rotation() = ToProto(rigid.rotation());
  return proto;
}

transform::Rigid3d ToRigid3(const proto::Rigid3d& rigid) {
  return transform::Rigid3d(ToEigen(rigid.translation()),
                            ToEigen(rigid.rotation()));
}

proto::Rigid3f ToProto(const transform::Rigid3f& rigid) {
  proto::Rigid3f proto;
  *proto.mutable_translation() = ToProto(rigid.translation());
  *proto.mutable_rotation() = ToProto(rigid.rotation());
  return proto;
}

proto::Vector2d ToProto(const Eigen::Vector2d& vector) {
  proto::Vector2d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  return proto;
}

proto::Vector3f ToProto(const Eigen::Vector3f& vector) {
  proto::Vector3f proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

proto::Vector4f ToProto(const Eigen::Vector4f& vector) {
  proto::Vector4f proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  proto.set_t(vector.w());
  return proto;
}

proto::Vector3d ToProto(const Eigen::Vector3d& vector) {
  proto::Vector3d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

proto::Quaternionf ToProto(const Eigen::Quaternionf& quaternion) {
  proto::Quaternionf proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}

proto::Quaterniond ToProto(const Eigen::Quaterniond& quaternion) {
  proto::Quaterniond proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}

}  // namespace transform
}  // namespace vins
}  // namespace xslam

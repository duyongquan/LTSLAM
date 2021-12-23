//
// Created by quan on 2021/12/10.
//


#include "xslam/ceres/pose_data_types.h"

namespace xslam {
namespace ceres {
namespace example {

/****************************************** 2D ******************************************/
std::istream& operator>>(std::istream& input, Pose2d& pose)
{
    input >> pose.x >> pose.y >> pose.yaw_radians;
    // Normalize the angle between -pi to pi.
    pose.yaw_radians = NormalizeAngle(pose.yaw_radians);
    return input;
}

std::istream& operator>>(std::istream& input, Constraint2d& constraint)
{
    input >> constraint.id_begin >> constraint.id_end >> constraint.x >>
          constraint.y >> constraint.yaw_radians >>
          constraint.information(0, 0) >> constraint.information(0, 1) >>
          constraint.information(0, 2) >> constraint.information(1, 1) >>
          constraint.information(1, 2) >> constraint.information(2, 2);

    // Set the lower triangular part of the information matrix.
    constraint.information(1, 0) = constraint.information(0, 1);
    constraint.information(2, 0) = constraint.information(0, 2);
    constraint.information(2, 1) = constraint.information(1, 2);

    // Normalize the angle between -pi to pi.
    constraint.yaw_radians = NormalizeAngle(constraint.yaw_radians);
    return input;
}

/****************************************** 3D ******************************************/
std::istream& operator>>(std::istream& input, Pose3d& pose) {
    input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >>
          pose.q.y() >> pose.q.z() >> pose.q.w();
    // Normalize the quaternion to account for precision loss due to
    // serialization.
    pose.q.normalize();
    return input;
}


std::istream& operator>>(std::istream& input, Constraint3d& constraint)
{
    Pose3d& t_be = constraint.t_be;
    input >> constraint.id_begin >> constraint.id_end >> t_be;

    for (int i = 0; i < 6 && input.good(); ++i) {
        for (int j = i; j < 6 && input.good(); ++j) {
            input >> constraint.information(i, j);
            if (i != j) {
                constraint.information(j, i) = constraint.information(i, j);
            }
        }
    }
    return input;
}

} // namespace example
} // namespace ceres
} // namespace xslam
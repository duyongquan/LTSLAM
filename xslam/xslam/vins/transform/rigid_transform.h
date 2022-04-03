#ifndef XSLAM_VINS_TRANSFORM_RIGID_TRANSFORM_H_
#define XSLAM_VINS_TRANSFORM_RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "xslam/vins/common/lua_parameter_dictionary.h"
#include "xslam/vins/common/math.h"
#include "xslam/vins/common/port.h"

namespace xslam {
namespace vins {
namespace transform {

template <typename FloatType>
class Rigid2 
{
public:
    using Vector = Eigen::Matrix<FloatType, 2, 1>;
    using Rotation2D = Eigen::Rotation2D<FloatType>;

    Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
    Rigid2(const Vector& translation, const Rotation2D& rotation)
        : translation_(translation), rotation_(rotation) {}
    Rigid2(const Vector& translation, const double rotation)
        : translation_(translation), rotation_(rotation) {}

    static Rigid2 Rotation(const double rotation) 
    {
        return Rigid2(Vector::Zero(), rotation);
    }

    static Rigid2 Rotation(const Rotation2D& rotation) 
    {
        return Rigid2(Vector::Zero(), rotation);
    }

    static Rigid2 Translation(const Vector& vector) 
    {
        return Rigid2(vector, Rotation2D::Identity());
    }

    static Rigid2<FloatType> Identity() { return Rigid2<FloatType>(); }

    template <typename OtherType>
    Rigid2<OtherType> cast() const 
    {
        return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                                rotation_.template cast<OtherType>());
    }

    const Vector& translation() const { return translation_; }

    Rotation2D rotation() const { return rotation_; }

    double normalized_angle() const 
    {
        return common::NormalizeAngleDifference(rotation().angle());
    }

    Rigid2 inverse() const 
    {
        const Rotation2D rotation = rotation_.inverse();
        const Vector translation = -(rotation * translation_);
        return Rigid2(translation, rotation);
    }

private:
    Vector translation_;
    Rotation2D rotation_;
};

template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs, const Rigid2<FloatType>& rhs) 
{
    return Rigid2<FloatType>(
        lhs.rotation() * rhs.translation() + lhs.translation(),
        lhs.rotation() * rhs.rotation());
}

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
    return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os, const xslam::vins::transform::Rigid2<T>& rigid) 
{
    os << rigid.DebugString();
    return os;
}

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

template <typename FloatType>
class Rigid3 
{
public:
    using Vector = Eigen::Matrix<FloatType, 3, 1>;
    using Quaternion = Eigen::Quaternion<FloatType>;
    using AngleAxis = Eigen::AngleAxis<FloatType>;

    Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
    Rigid3(const Vector& translation, const Quaternion& rotation)
        : translation_(translation), rotation_(rotation) {}

    Rigid3(const Vector& translation, const AngleAxis& rotation)
        : translation_(translation), rotation_(rotation) {}

    static Rigid3 Rotation(const AngleAxis& angle_axis) 
    {
        return Rigid3(Vector::Zero(), Quaternion(angle_axis));
    }

    static Rigid3 Rotation(const Quaternion& rotation) 
    {
        return Rigid3(Vector::Zero(), rotation);
    }

    static Rigid3 Translation(const Vector& vector) 
    {
        return Rigid3(vector, Quaternion::Identity());
    }

    static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation,
                            const std::array<FloatType, 3>& translation) 
    {
        return Rigid3(Eigen::Map<const Vector>(translation.data()),
                      Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                                  rotation[2], rotation[3]));
    }

    static Rigid3<FloatType> Identity() { return Rigid3<FloatType>(); }

    template <typename OtherType>
    Rigid3<OtherType> cast() const 
    {
        return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                                rotation_.template cast<OtherType>());
    }

    const Vector& translation() const { return translation_; }
    const Quaternion& rotation() const { return rotation_; }

    Rigid3 inverse() const 
    {
        const Quaternion rotation = rotation_.conjugate();
        const Vector translation = -(rotation * translation_);
        return Rigid3(translation, rotation);
    }

    bool IsValid() const 
    {
        return !std::isnan(translation_.x()) && 
               !std::isnan(translation_.y()) &&
               !std::isnan(translation_.z()) &&
               std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
    }

private:
    Vector translation_;
    Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs, const Rigid3<FloatType>& rhs) 
{
    return Rigid3<FloatType>(
        lhs.rotation() * rhs.translation() + lhs.translation(),
        (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid, const typename Rigid3<FloatType>::Vector& point) 
{
    return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os, const xslam::vins::transform::Rigid3<T>& rigid) 
{
    os << rigid.DebugString();
    return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

// Returns an transform::Rigid3d given a 'dictionary' containing 'translation'
// (x, y, z) and 'rotation' which can either we an array of (roll, pitch, yaw)
// or a dictionary with (w, x, y, z) values as a quaternion.
Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary);

}  // namespace transform
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_TRANSFORM_RIGID_TRANSFORM_H_

#include "xslam/vins/transform/rigid_transform.h"

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "xslam/vins/common/lua_parameter_dictionary.h"
#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace transform {

namespace {

Eigen::Vector3d TranslationFromDictionary(
    common::LuaParameterDictionary* dictionary) 
{
    const std::vector<double> translation = dictionary->GetArrayValuesAsDoubles();
    CHECK_EQ(3, translation.size()) << "Need (x, y, z) for translation.";
    return Eigen::Vector3d(translation[0], translation[1], translation[2]);
}

}  // namespace

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch, const double yaw) 
{
    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    return yaw_angle * pitch_angle * roll_angle;
}

transform::Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary) 
{

    const Eigen::Vector3d translation =
        TranslationFromDictionary(dictionary->GetDictionary("translation").get());

    auto rotation_dictionary = dictionary->GetDictionary("rotation");
    if (rotation_dictionary->HasKey("w")) {
        const Eigen::Quaterniond rotation(rotation_dictionary->GetDouble("w"),
                                          rotation_dictionary->GetDouble("x"),
                                          rotation_dictionary->GetDouble("y"),
                                          rotation_dictionary->GetDouble("z"));
        CHECK_NEAR(rotation.norm(), 1., 1e-9);
        return transform::Rigid3d(translation, rotation);
    } else {
        const std::vector<double> rotation =
            rotation_dictionary->GetArrayValuesAsDoubles();
        CHECK_EQ(3, rotation.size()) << "Need (roll, pitch, yaw) for rotation.";
        return transform::Rigid3d(
            translation, RollPitchYaw(rotation[0], rotation[1], rotation[2]));
    }
}

}  // namespace transform
}  // namespace vins
}  // namespace xslam

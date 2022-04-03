#include <string>
#include <vector>

#include "xslam/vins/common/config.h"
#include "xslam/vins/common/configuration_file_resolver.h"
#include "xslam/vins/common/lua_parameter_dictionary.h"

#include "gtest/gtest.h"

#if 0
namespace xslam {
namespace vins {
namespace common {
namespace {

TEST(ConfigurationFilesTest, ValidateMapBuilderOptions) {
  const std::string kCode = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      return MAP_BUILDER)text";
  EXPECT_NO_FATAL_FAILURE({
    auto file_resolver =
        std::make_unique< ::xslam::common::ConfigurationFileResolver>(
            std::vector<std::string>{
                std::string(::xslam::common::kVinsSourceDirectory) + "/configuration"});
    ::xslam::common::LuaParameterDictionary lua_parameter_dictionary(
        kCode, std::move(file_resolver));
  });
}

}  // namespace
}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif 

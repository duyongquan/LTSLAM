#ifndef XSLAM_COMMON_CONFIGURATION_FILE_RESOLVER_H_
#define XSLAM_COMMON_CONFIGURATION_FILE_RESOLVER_H_

#include <vector>

#include "xslam/vins/common/lua_parameter_dictionary.h"
#include "xslam/vins/common/port.h"

namespace xslam {
namespace vins {
namespace common {

// A 'FileResolver' for the 'LuaParameterDictionary' that reads files from disk.
// It searches the 'configuration_files_directories' in order to find the
// requested filename. The last place searched is always the
// 'configuration_files/' directory installed with Cartographer. It contains
// reasonable configuration for the various Cartographer components which
// provide a good starting ground for new platforms.
class ConfigurationFileResolver : public FileResolver 
{
public:
    explicit ConfigurationFileResolver(
        const std::vector<std::string>& configuration_files_directories);

    std::string GetFullPathOrDie(const std::string& basename) override;
    std::string GetFileContentOrDie(const std::string& basename) override;

private:
    std::vector<std::string> configuration_files_directories_;
};

}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_COMMON_CONFIGURATION_FILE_RESOLVER_H_

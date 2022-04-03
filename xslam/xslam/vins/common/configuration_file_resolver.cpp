#include "xslam/vins/common/configuration_file_resolver.h"

#include <fstream>
#include <iostream>
#include <streambuf>

#include "xslam/vins/common/configuration_file_resolver.h"
#include "xslam/vins/common/config.h"
#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace common {

ConfigurationFileResolver::ConfigurationFileResolver(
    const std::vector<std::string>& configuration_files_directories)
    : configuration_files_directories_(configuration_files_directories) {
  configuration_files_directories_.push_back(kConfigurationFilesDirectory);
}

std::string ConfigurationFileResolver::GetFullPathOrDie(
    const std::string& basename) {
  for (const auto& path : configuration_files_directories_) {
    const std::string filename = path + "/" + basename;
    std::ifstream stream(filename.c_str());
    if (stream.good()) {
      LOG(INFO) << "Found '" << filename << "' for '" << basename << "'.";
      return filename;
    }
  }
  LOG(FATAL) << "File '" << basename << "' was not found.";
}

std::string ConfigurationFileResolver::GetFileContentOrDie(
    const std::string& basename) {
  CHECK(!basename.empty()) << "File basename cannot be empty." << basename;
  const std::string filename = GetFullPathOrDie(basename);
  std::ifstream stream(filename.c_str());
  return std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
}

}  // namespace common
}  // namespace vins
}  // namespace xslam

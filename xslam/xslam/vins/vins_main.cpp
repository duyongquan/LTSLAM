#include "xslam/vins/vins.h"
#include "xslam/vins/common/config.h"
#include "xslam/vins/common/logging.h"
#include "gflags/gflags.h"

#include <vector>
#include <string>
#include <memory>

DEFINE_string(configuration_directory, xslam::vins::common::kConfigurationFilesDirectory,
              "First directory in which configuration files are searched, "
              "second is always the XSLAM installation to allow "
              "including files from there.");

DEFINE_string(configuration_basename, "vins.lua",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace xslam {
namespace vins {
namespace {

void Run(int argc, char **argv)
{
    // Create VINS system instance and load configuration file.
    auto vins_options = LoadOptions(FLAGS_configuration_directory, 
        FLAGS_configuration_basename);
    std::shared_ptr<xslam::vins::VINSSystem> system = 
        std::make_shared<xslam::vins::VINSSystem>(vins_options);

    // Comamand parsing.
    // if (argc == 1) {
    //     return system->ShowHelp();
    // }

    // Run finish.
    system->Shutdown();
}

} // namespace
} // namespace vins
} // namespace xslam

int main(int argc, char **argv)
{
    xslam::vins::common::InitializeGlog(argv);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty()) 
        << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";

    xslam::vins::Run(argc, argv);
    return 0;
}
#include "xslam/vins/vins.h"
#include "xslam/vins/common/logging.h"
#include "gflags/gflags.h"

#include <vector>
#include <string>
#include <memory>

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the XSLAM installation to allow "
              "including files from there.");

DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace xslam {
namespace vins {
namespace {

void Run(int argc, char **argv)
{
    // Create VINS system instance and load configuration file.
    std::string config_filename = " ";
    std::shared_ptr<xslam::vins::VINSSystem> system = 
        std::make_shared<xslam::vins::VINSSystem>(config_filename);

    // Comamand parsing.
    if (argc == 1) {
        return system->ShowHelp();
    }

    // Run finish.
    system->Shutdown();
}

} // namespace
} // namespace vins
} // namespace xslam

int main(int argc, char **argv)
{
    xslam::common::InitializeGlog(argv);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty()) 
        << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";

    xslam::vins::Run(argc, argv);
    return 0;
}
#include "xslam/vins/vins.h"
#include "xslam/vins/common/logging.h"

#include <vector>
#include <string>
#include <memory>

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
    xslam::vins::Run(argc, argv);
    return 0;
}
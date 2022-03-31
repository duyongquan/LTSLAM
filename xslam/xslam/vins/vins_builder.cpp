#include "xslam/vins/vins_builder.h"


namespace xslam {
namespace vins {

VinsBuilder::VinsBuilder(const proto::VinsBuilderOptions& options)
    : options_(options)
{

}


std::unique_ptr<VinsBuilderInterface> CreateVinsBuilder(
    const proto::VinsBuilderOptions& options)
{ 
    return std::make_unique<VinsBuilder>(options);
}

} // namespace vins
} // namespace xslam
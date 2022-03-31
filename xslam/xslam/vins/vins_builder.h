#ifndef XSLAM_VINS_VINS_BUILDER_H_
#define XSLAM_VINS_VINS_BUILDER_H_

#include <memory>

#include "xslam/vins/vins_builder_interface.h"
#include "xslam/vins/estimator/proto/vins_builder_options.pb.h"

namespace xslam {
namespace vins {

class VinsBuilder : public VinsBuilderInterface 
{
public:
    explicit VinsBuilder(const proto::VinsBuilderOptions &options);
    ~VinsBuilder() override {} 

  
    estimator::Estimator* estimator() override 
    {
        return estimator_.get();
    }

private:
    const proto::VinsBuilderOptions options_;

    std::unique_ptr<estimator::Estimator> estimator_;

};

std::unique_ptr<VinsBuilderInterface> CreateVinsBuilder(
    const proto::VinsBuilderOptions& options);

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_BUILDER_H_
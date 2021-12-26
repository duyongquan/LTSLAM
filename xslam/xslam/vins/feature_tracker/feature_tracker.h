#ifndef XSLAM_VINS_FEATURE_TRACKER_H
#define XSLAM_VINS_FEATURE_TRACKER_H

#include <thread>
#include <string>

namespace xslam {
namespace feature_tracker {
    
class FeatureTracker 
{
public:
    FeatureTracker(const std::string& filename);

};

} // namespace feature_tracker
} // namespace xslam 

#endif // XSLAM_VINS_FEATURE_TRACKER_H
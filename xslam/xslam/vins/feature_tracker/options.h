#ifndef XSLAM_VINS_OPTIONS_H
#define XSLAM_VINS_OPTIONS_H

#include <string>

#include "xslam/vins/common/lua_parameter_dictionary.h"

namespace xslam {
namespace vins {
namespace feature_tracker {

struct FeatureTrackerOptions
{
    int row;
    int col;
    int focal_length;
    int num_cam = 1;

    int max_count;
    int min_distance;
    int window_size;
    int frequency;
    int show_track;
    int stereo_track;
    int equalize;
    int fisheye;
    bool publish_this_frame;
    double fundamental_threshold;
};

} // namespace feature_tracker
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_OPTIONS_H
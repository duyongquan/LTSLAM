#ifndef XSLAM_COMMON_MESSAGES_CHANNEL_POINT32_H
#define XSLAM_COMMON_MESSAGES_CHANNEL_POINT32_H

#include "xslam/vins/common/port.h"

#include <vector>
#include <string>

namespace xslam {
namespace vins {
namespace common {
namespace messages {

struct ChannelFloat32
{
    // The channel name should give semantics of the channel (e.g.
    // "intensity" instead of "value").
    std::string name;

    // The values array should be 1-1 with the elements of the associated
    // PointCloud.
    std::vector<double> values;
};

}  // messages
}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_COMMON_MESSAGES_CHANNEL_POINT32_H
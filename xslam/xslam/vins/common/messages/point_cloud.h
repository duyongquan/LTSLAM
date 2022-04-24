#ifndef XSLAM_COMMON_MESSAGES_POINT_CLOUD_H
#define XSLAM_COMMON_MESSAGES_POINT_CLOUD_H

#include "xslam/vins/common/port.h"
#include "xslam/vins/common/time.h"
#include "xslam/vins/common/messages/header.h"
#include "xslam/vins/common/messages/point32.h"
#include "xslam/vins/common/messages/channel_float32.h"

#include <string>
#include <vector>

namespace xslam {
namespace vins {
namespace common {
namespace messages {

struct PointCloud
{
    Header header;
    std::vector<Point32> points;
    std::vector<ChannelFloat32> channels;
};

}  // messages
}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_COMMON_MESSAGES_POINT_CLOUD_H
#ifndef XSLAM_COMMON_MESSAGES_POINT32_H
#define XSLAM_COMMON_MESSAGES_POINT32_H

#include "xslam/vins/common/port.h"

namespace xslam {
namespace vins {
namespace common {
namespace messages {

struct Point32
{
  double x;
  double y;
  double z;
};

}  // messages
}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_COMMON_MESSAGES_POINT32_H
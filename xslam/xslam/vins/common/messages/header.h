#ifndef XSLAM_COMMON_MESSAGES_HEADER_H
#define XSLAM_COMMON_MESSAGES_HEADER_H

#include "xslam/vins/common/port.h"
#include "xslam/vins/common/time.h"

#include <string>

namespace xslam {
namespace vins {
namespace common {
namespace messages {

struct Header
{
  uint32 seq;
  Time stamp;
  std::string frame_id;
};

}  // messages
}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_COMMON_MESSAGES_HEADER_H
#ifndef XSLAM_VINS_COMMON_PORT_H
#define XSLAM_VINS_COMMON_PORT_H

#include <cinttypes>
#include <cmath>
#include <string>

namespace xslam {
namespace vins {

using int8   = int8_t;
using int16  = int16_t;
using int32  = int32_t;
using int64  = int64_t;
using uint8  = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

namespace common {

inline int RoundToInt(const float x) 
{
    return std::lround(x); 
}

inline int RoundToInt(const double x) 
{ 
    return std::lround(x); 
}

inline int64 RoundToInt64(const float x) 
{ 
    return std::lround(x); 
}

inline int64 RoundToInt64(const double x) 
{ 
    return std::lround(x); 
}

}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_COMMON_PORT_H

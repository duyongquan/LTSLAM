#include "xslam/vins/common/time.h"

#include <time.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace common {

Duration FromSeconds(const double seconds) 
{
    return std::chrono::duration_cast<Duration>(
        std::chrono::duration<double>(seconds));
}

double ToSeconds(const Duration duration) 
{
    return std::chrono::duration_cast<
        std::chrono::duration<double>>(duration).count();
}

double ToSeconds(const std::chrono::steady_clock::duration duration) 
{
    return std::chrono::duration_cast<
        std::chrono::duration<double>>(duration).count();
}

Time FromUniversal(const int64 ticks) 
{ 
    return Time(Duration(ticks)); 
}

int64 ToUniversal(const Time time) 
{ 
    return time.time_since_epoch().count(); 
}

std::ostream& operator<<(std::ostream& os, const Time time) 
{
    os << std::to_string(ToUniversal(time));
    return os;
}

common::Duration FromMilliseconds(const int64 milliseconds) 
{
    return std::chrono::duration_cast<Duration>(
        std::chrono::milliseconds(milliseconds));
}

double GetThreadCpuTimeSeconds() 
{
    return 0.;
}

}  // namespace common
}  // namespace vins
}  // namespace xslam

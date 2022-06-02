#include "stdout_consumer.hpp"

#include <iostream>
#include <iomanip>

namespace tutorial {
namespace common {
namespace log {

std::ostream& StdoutConsumer::get_stream(const Log::Entry& entry)
{
    (void) entry;
    return std::cout;
}

} // namespace log
} // namespace common
} // namespace tutorial

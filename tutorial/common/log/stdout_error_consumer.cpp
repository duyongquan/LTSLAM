#include "stdout_error_consumer.hpp"

#include <iomanip>


namespace tutorial {
namespace common {
namespace log {

void StdoutErrConsumer::stderr_threshold(const Log::Kind& kind)
{
    stderr_threshold_ = kind;
}

Log::Kind StdoutErrConsumer::stderr_threshold() const
{
    return stderr_threshold_;
}

std::ostream& StdoutErrConsumer::get_stream(const Log::Entry& entry)
{
    // If Log::Kind is stderr_threshold_ or more severe, then use STDERR, else use STDOUT
    if (entry.kind <= stderr_threshold_)
    {
        return std::cerr;
    }
    return std::cout;
}

} // namespace log
} // namespace common
} // namespace tutorial
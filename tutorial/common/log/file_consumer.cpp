#include "file_consumer.hpp"

#include <iomanip>

namespace tutorial {
namespace common {
namespace log {

FileConsumer::FileConsumer()
    : FileConsumer("output.log")
{
}

FileConsumer::FileConsumer(
        const std::string& filename,
        bool append)
    : output_file_(filename)
    , append_(append)
{
    if (append_)
    {
        file_.open(output_file_, std::ios::out | std::ios::app);
    }
    else
    {
        file_.open(output_file_, std::ios::out);
    }
}

FileConsumer::~FileConsumer()
{
    file_.close();
}

std::ostream& FileConsumer::get_stream(const Log::Entry& entry)
{
    (void) entry;
    return file_;
}

} // namespace log
} // namespace common
} // namespace tutorial


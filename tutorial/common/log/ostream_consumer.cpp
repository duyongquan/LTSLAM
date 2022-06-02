#include "ostream_consumer.hpp"

namespace tutorial {
namespace common {
namespace log {

void OStreamConsumer::Consume(
        const Log::Entry& entry)
{
    std::ostream& stream = get_stream(entry);
    print_timestamp(stream, entry, true);
    print_header(stream, entry, true);
    print_message(stream, entry, true);
    print_context(stream, entry, true);
    print_new_line(stream, true);
    stream.flush();
}

} // namespace log
} // namespace common
} // namespace tutorial
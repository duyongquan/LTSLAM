#ifndef TUTORIAL_COMMON_LOG_STDOUT_CONSUMER_HPP
#define TUTORIAL_COMMON_LOG_STDOUT_CONSUMER_HPP

#include "log.hpp"
#include "ostream_consumer.hpp"

namespace tutorial {
namespace common {
namespace log {

class StdoutConsumer : public OStreamConsumer
{
public:

    virtual ~StdoutConsumer() = default;

private:

    /** \internal
     * Called by Log consume to get the correct stream
     * @param Log::Entry to consume.
     */
    virtual std::ostream& get_stream(const Log::Entry& entry) override;
};

} // namespace log
} // namespace common
} // namespace tutorial

#endif // ifndef TUTORIAL_COMMON_LOG_STDOUT_CONSUMER_HPP

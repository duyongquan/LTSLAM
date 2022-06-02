#ifndef TUTORIAL_COMMON_OSTREAM_CONSUMER_HPP
#define TUTORIAL_COMMON_OSTREAM_CONSUMER_HPP

#include <iostream>

#include "log.hpp"

namespace tutorial {
namespace common {
namespace log {

/**
 * Log consumer interface for std::ostream objects
 *
 * @file OStreamConsumer.hpp
 */
class OStreamConsumer : public LogConsumer
{
public:

    virtual ~OStreamConsumer() = default;

    /** \internal
     * Called by Log to ask us to consume the Entry.
     * @param Log::Entry to consume.
     */
    void Consume(const Log::Entry& entry) override;

protected:

    /** \internal
     * Called by Log consume to get the correct stream
     * @param Log::Entry to consume.
     */
    virtual std::ostream& get_stream(const Log::Entry& entry) = 0;
};

} // namespace log
} // namespace common
} // namespace tutorial

#endif // ifndef TUTORIAL_COMMON_OSTREAM_CONSUMER_HPP

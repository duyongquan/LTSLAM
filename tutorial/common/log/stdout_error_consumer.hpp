#ifndef TUTORIAL_COMMON_LOG_STDOUT_ERROR_CONSUMER_HPP
#define TUTORIAL_COMMON_LOG_STDOUT_ERROR_CONSUMER_HPP

#include "log.hpp"
#include "ostream_consumer.hpp"

#include <iostream>

namespace tutorial {
namespace common {
namespace log {

/**
 * Log consumer that writes the log events with a Log::Kind equal to or more severe than a threshold (defaults to
 * Log::Kind::Warning) to the STDERR, and events with a Log::Kind less severe than the threshold to the STDOUT.
 *
 * @file StdoutErrConsumer.hpp
 */
class StdoutErrConsumer : public OStreamConsumer
{
public:

    virtual ~StdoutErrConsumer() = default;

    /**
     * @brief Set the stderr_threshold to a Log::Kind.
     * This threshold decides which log messages are output on STDOUT, and which are output to STDERR.
     * Log messages with a Log::Kind equal to or more severe than the stderr_threshold are output to STDERR using
     * std::cerr.
     * Log messages with a Log::Kind less severe than the stderr_threshold are output to STDOUT using
     * std::cout.
     * @param kind The Log::Kind to which stderr_threshold is set.
     */
    virtual void stderr_threshold(const Log::Kind& kind);

    /**
     * @brief Retrieve the stderr_threshold.
     * @return The Log::Kind to which stderr_threshold is set.
     */
    virtual Log::Kind stderr_threshold() const;

    /**
     * @brief Default value of stderr_threshold.
     */
    static constexpr Log::Kind STDERR_THRESHOLD_DEFAULT = Log::Kind::Warning;

protected:

    /** \internal
     * Called by Log consume to get the correct stream
     * @param Log::Entry to consume.
     */
    virtual std::ostream& get_stream(const Log::Entry& entry) override;

private:

    Log::Kind stderr_threshold_ = STDERR_THRESHOLD_DEFAULT;

};

} // namespace log
} // namespace common
} // namespace tutorial

#endif // ifndef TUTORIAL_COMMON_LOG_STDOUT_ERROR_CONSUMER_HPP
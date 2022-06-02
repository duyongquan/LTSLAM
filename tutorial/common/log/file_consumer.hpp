#ifndef TUTORIAL_CPP_COMMON_FILE_CONSUMER_HPP
#define TUTORIAL_CPP_COMMON_FILE_CONSUMER_HPP

#include "log.hpp"
#include "ostream_consumer.hpp"

#include <fstream>

namespace tutorial {
namespace common {
namespace log {

/**
 * Log consumer that writes the log events to a file.
 *
 * @file FileConsumer.hpp
 */
class FileConsumer : public OStreamConsumer
{
public:

    //! Default constructor: filename = "output.log", append = false.
    FileConsumer();

    /** Constructor with parameters.
     * @param filename path of the output file where the log will be wrote.
     * @param append indicates if the consumer must append the content in the filename.
     */
    FileConsumer(const std::string& filename, bool append = false);

    virtual ~FileConsumer();

private:

    /** \internal
     * Called by Log consume to get the correct stream
     * @param entry Log::Entry to consume.
     */
    virtual std::ostream& get_stream(const Log::Entry& entry) override;

    std::string output_file_;
    std::ofstream file_;
    bool append_;
};

} // namespace log
} // namespace common
} // namespace tutorial

#endif // TUTORIAL_CPP_COMMON_FILE_CONSUMER_HPP

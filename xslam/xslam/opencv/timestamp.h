#ifndef XSLAM_TUTORIAL_TIMESTAMP_H
#define XSLAM_TUTORIAL_TIMESTAMP_H

#include <string>

namespace xslam {
namespace opencv {

class Timestamp
{
public:
    enum tOptions
    {
        NONE = 0,
        CURRENT_TIME = 0x1,
        ZERO = 0x2
    };
  
  
	Timestamp(Timestamp::tOptions option = NONE);
	virtual ~Timestamp(void);

    // Says if the timestamp is "empty": seconds and usecs are both 0, as 
    // when initiated with the ZERO flag
    bool empty() const;

	// Sets this instance to the current time
	void setToCurrentTime();

	// Sets the timestamp from seconds and microseconds
	inline void setTime(unsigned long secs, unsigned long usecs)
    {
		m_secs = secs;
		m_usecs = usecs;
	}
	
	// Returns the timestamp in seconds and microseconds
	inline void getTime(unsigned long &secs, unsigned long &usecs) const
	{
	  secs = m_secs;
	  usecs = m_usecs;
	}

	// Sets the timestamp from a string with the time in seconds
	void setTime(const std::string &stime);
	
	// Sets the timestamp from a number of seconds from the epoch
	void setTime(double s);
	
	// Returns this timestamp as the number of seconds in (long) float format
	double getFloatTime() const;

	// Returns this timestamp as the number of seconds in fixed length string format
	std::string getStringTime() const;

	// Returns the difference in seconds between this timestamp (greater) and t (smaller)
	// If the order is swapped, a negative number is returned
	double operator- (const Timestamp &t) const;

	// Returns a copy of this timestamp + s seconds + us microseconds
	Timestamp plus(unsigned long s, unsigned long us) const;

    // Returns a copy of this timestamp - sstd::
	Timestamp minus(unsigned long s, unsigned long us) const;
  
	Timestamp& operator+= (double s);

    // Substracts s seconds to this timestamp and returns a reference to itself
    Timestamp& operator-= (double s);

	// Returns a copy of this timestamp + s seconds
	Timestamp operator+ (double s) const;

	// Returns a copy of this timestamp - s seconds
	Timestamp operator- (double s) const;

	// Returns whether this timestamp is at the future of t
	bool operator> (const Timestamp &t) const;

	// Returns whether this timestamp is at the future of (or is the same as) t
	bool operator>= (const Timestamp &t) const;

	// Returns whether this timestamp and t represent the same instant
	bool operator== (const Timestamp &t) const;

	// Returns whether this timestamp is at the past of t
	bool operator< (const Timestamp &t) const;

	// Returns whether this timestamp is at the past of (or is the same as) t
	bool operator<= (const Timestamp &t) const;

    // Returns the timestamp in a human-readable string
    std::string Format(bool machine_friendly = false) const;

	// Returns a string version of the elapsed time in seconds, with the format
	// xd hh:mm:ss, hh:mm:ss, mm:ss or s.us
	static std::string Format(double s);
	
protected:
    // Seconds
	unsigned long m_secs;	// seconds
	// Microseconds
	unsigned long m_usecs;	// microseconds
};

} // namespace opencv
} // namespace xslam

#endif // XSLAM_TUTORIAL_TIMESTAMP_H
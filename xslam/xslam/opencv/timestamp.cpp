#include "xslam/opencv/timestamp.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <sys/time.h>

namespace xslam {
namespace opencv {

Timestamp::Timestamp(Timestamp::tOptions option)
{
    if(option & CURRENT_TIME)
        setToCurrentTime();
    else if(option & ZERO)
        setTime(0.);
}

Timestamp::~Timestamp(void)
{
}

bool Timestamp::empty() const
{
    return m_secs == 0 && m_usecs == 0;
}

void Timestamp::setToCurrentTime()
{
	
	struct timeval now;
	gettimeofday(&now, NULL);
	m_secs = now.tv_sec;
	m_usecs = now.tv_usec;
}

void Timestamp::setTime(const std::string &stime)
{
	std::string::size_type p = stime.find('.');
	if(p == std::string::npos)
    {
		m_secs = atol(stime.c_str());
		m_usecs = 0;
	}else{
		m_secs = atol(stime.substr(0, p).c_str());
		
		std::string s_usecs = stime.substr(p+1, 6);
		m_usecs = atol(stime.substr(p+1).c_str());
		m_usecs *= (unsigned long)pow(10.0, double(6 - s_usecs.length()));
	}
}

void Timestamp::setTime(double s)
{
    m_secs = (unsigned long)s;
    m_usecs = (s - (double)m_secs) * 1e6;
}

double Timestamp::getFloatTime() const 
{
	return double(m_secs) + double(m_usecs)/1000000.0;
}

std::string Timestamp::getStringTime() const 
{
	char buf[32];
	sprintf(buf, "%.6lf", this->getFloatTime());
	return std::string(buf);
}

double Timestamp::operator- (const Timestamp &t) const 
{
	return this->getFloatTime() - t.getFloatTime();
}

Timestamp& Timestamp::operator+= (double s)
{
    *this = *this + s;
    return *this;
}

Timestamp& Timestamp::operator-= (double s)
{
    *this = *this - s;
    return *this;
}

Timestamp Timestamp::operator+ (double s) const
{
	unsigned long secs = (long)floor(s);
	unsigned long usecs = (long)((s - (double)secs) * 1e6);

    return this->plus(secs, usecs);
}

Timestamp Timestamp::plus(unsigned long secs, unsigned long usecs) const
{
    Timestamp t;

	const unsigned long max = 1000000ul;

	if(m_usecs + usecs >= max)
		t.setTime(m_secs + secs + 1, m_usecs + usecs - max);
	else
		t.setTime(m_secs + secs, m_usecs + usecs);
	
	return t;
}

Timestamp Timestamp::operator- (double s) const
{
	unsigned long secs = (long)floor(s);
	unsigned long usecs = (long)((s - (double)secs) * 1e6);

	return this->minus(secs, usecs);
}

Timestamp Timestamp::minus(unsigned long secs, unsigned long usecs) const
{
    Timestamp t;

	const unsigned long max = 1000000ul;

	if(m_usecs < usecs)
		t.setTime(m_secs - secs - 1, max - (usecs - m_usecs));
	else
		t.setTime(m_secs - secs, m_usecs - usecs);
	
	return t;
}

bool Timestamp::operator> (const Timestamp &t) const
{
	if(m_secs > t.m_secs) return true;
	else if(m_secs == t.m_secs) return m_usecs > t.m_usecs;
	else return false;
}

bool Timestamp::operator>= (const Timestamp &t) const
{
	if(m_secs > t.m_secs) return true;
	else if(m_secs == t.m_secs) return m_usecs >= t.m_usecs;
	else return false;
}

bool Timestamp::operator< (const Timestamp &t) const
{
	if(m_secs < t.m_secs) return true;
	else if(m_secs == t.m_secs) return m_usecs < t.m_usecs;
	else return false;
}

bool Timestamp::operator<= (const Timestamp &t) const
{
	if(m_secs < t.m_secs) return true;
	else if(m_secs == t.m_secs) return m_usecs <= t.m_usecs;
	else return false;
}

bool Timestamp::operator== (const Timestamp &t) const
{
	return(m_secs == t.m_secs && m_usecs == t.m_usecs);
}


std::string Timestamp::Format(bool machine_friendly) const 
{
    struct tm tm_time;

    time_t t = (time_t)getFloatTime();

    localtime_r(&t, &tm_time);
    
    char buffer[128];
    
    if(machine_friendly)
    {
        strftime(buffer, 128, "%Y%m%d_%H%M%S", &tm_time);
    }
    else
    {
        strftime(buffer, 128, "%c", &tm_time); // Thu Aug 23 14:55:02 2001
    }
    
    return std::string(buffer);
}

std::string Timestamp::Format(double s) 
{
	int days = int(s / (24. * 3600.0));
	s -= days * (24. * 3600.0);
	int hours = int(s / 3600.0);
	s -= hours * 3600;
	int minutes = int(s / 60.0);
	s -= minutes * 60;
	int seconds = int(s);
	int ms = int((s - seconds)*1e6);

	std::stringstream ss;
	ss.fill('0');
	bool b;
	if((b = (days > 0))) ss << days << "d ";
	if((b = (b || hours > 0))) ss << std::setw(2) << hours << ":";
	if((b = (b || minutes > 0))) ss << std::setw(2) << minutes << ":";
	if(b) ss << std::setw(2);
	ss << seconds;
	if(!b) ss << "." << std::setw(6) << ms;

	return ss.str();
}

} // namespace opencv
} // namespace xslam
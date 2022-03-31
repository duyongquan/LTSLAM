#ifndef XSLAM_VINS_COMMON_STRING_H
#define XSLAM_VINS_COMMON_STRING_H

#include <string>
#include <vector>

namespace xslam {
namespace vins {
namespace common {

// Format string by replacing embedded format specifiers with their respective
// values, see `printf` for more details. This is a modified implementation
// of Google's BSD-licensed StringPrintf function.
std::string StringPrintf(const char* format, ...);

// Replace all occurrences of `old_str` with `new_str` in the given string.
std::string StringReplace(const std::string& str, const std::string& old_str,
                          const std::string& new_str);

// Get substring of string after search key
std::string StringGetAfter(const std::string& str, const std::string& key);

// Split string into list of words using the given delimiters.
std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim);

// Check whether a string starts with a certain prefix.
bool StringStartsWith(const std::string& str, const std::string& prefix);

// Remove whitespace from string on both, left, or right sides.
void StringTrim(std::string* str);
void StringLeftTrim(std::string* str);
void StringRightTrim(std::string* str);

// Convert string to lower/upper case.
void StringToLower(std::string* str);
void StringToUpper(std::string* str);

// Check whether the sub-string is contained in the given string.
bool StringContains(const std::string& str, const std::string& sub_str);

}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif  // XSLAM_VINS_COMMON_STRING_H

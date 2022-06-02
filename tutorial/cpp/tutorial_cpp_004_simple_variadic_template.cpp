#include <iostream>
#include <string>
#include <cstdarg>

#include "glog/logging.h"

namespace tutorial {
namespace cpp {
namespace {

// 对于泛型编程来说，一种常见需求是，一个函数可以接受任意数量的参数，
// 没有模板时可以通过 [std::va_list](https://en.cppreference.com/w/cpp/utility/variadic/va_list) 实现
void RunCase1_Print(int n, ...)
{
    std::va_list args;
    std::cout << "begin" << std::endl;
    va_start(args, n);
    std::cout << "1: " << va_arg(args, double) << std::endl;
    std::cout << "2: " << va_arg(args, int) << std::endl;
    std::cout << "3: " << va_arg(args, const char*) << std::endl;
    std::cout << "4: " << va_arg(args, const char*) << std::endl;
    va_end(args);
    std::cout << "end" << std::endl;
}

void RunCase2_Print(const char* fmt, ...) 
{
    char buf[256];
    std::va_list args;
    va_start(args, fmt);
    vsnprintf(buf, 256, fmt, args);
    va_end(args);
    std::cout << buf << std::endl;
}

void Run()
{
    LOG(INFO) << "Run RunCase1_Print() :";
    RunCase1_Print(4, 3.14, 42, std::string{"hello"}.c_str(), "world");

    LOG(INFO) << "Run RunCase2_Print() :";
    RunCase2_Print("%.2f %d %s %s", 3.14, 42, std::string{"hello"}.c_str(), "world");
}

} // namespace
} // namespace cpp
} // namespace tutorial

int main(int argc, char **argv)
{
    LOG(INFO) << "Start tutorial cpp template ... ";;
    tutorial::cpp::Run();
    return 0;
}
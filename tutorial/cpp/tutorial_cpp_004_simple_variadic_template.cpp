#include <iostream>
#include <string>
#include <cstdarg>
#include <utility>

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

template <typename... Ts>
void f(Ts&&... ts) {}

// C++11 引入了可变参数模板，一个模板参数包可以匹配任意数量的模板参数
void RunCase3()
{
    f(3.14, 42, std::string{"hello"}, "world");
}

// 为了对参数包中每个参数做操作，需要展开参数包，一般的写法是，把参数包改成一个参数与一个参数包，
// 用只接受一个参数的函数来处理单参数，用这个函数本身处理剩余参数的参数包

template <typename T>
void RunCase4_Print(const T& t)
{
    std::cout << t << std::endl;
}

template <typename T, typename... Ts>
void RunCase4_Print_1(const T& t, Ts&&... ts) 
{
  RunCase4_Print(t);
  RunCase4_Print(std::forward<Ts>(ts)...);
}

void Run()
{
    LOG(INFO) << "Run RunCase1_Print() :";
    RunCase1_Print(4, 3.14, 42, std::string{"hello"}.c_str(), "world");

    LOG(INFO) << "Run RunCase2_Print() :";
    RunCase2_Print("%.2f %d %s %s", 3.14, 42, std::string{"hello"}.c_str(), "world");

    LOG(INFO) << "Run RunCase4_Print_1() :";
    // RunCase4_Print_1(3.14, 42, std::string{"hello"}, "world");

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
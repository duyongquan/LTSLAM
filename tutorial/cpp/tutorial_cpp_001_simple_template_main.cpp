#include <iostream>
#include <string>

#include "glog/logging.h"

// 最简单的例子如下。使用作用域运算符（::）表示指定使用全局命名空间中的 max 模板，
// 而非 [std::max](https://en.cppreference.com/w/cpp/algorithm/max)

template <typename T>
T max(T a, T b) 
{
  return b < a ? a : b;
}

namespace tutorial {
namespace cpp {
namespace {

// 函数模板示例
void RunCase1()
{
    LOG(INFO) << ::max(1, 3);       // 3
    LOG(INFO) << ::max(1.0, 3.14);  // 3.14
    std::string s1 = "mathematics";
    std::string s2 = "math";
    LOG(INFO) << ::max(s1, s2);  // mathematics
}

// Two-Phase Translation
void RunCase2()
{

}

void Run()
{
    RunCase1();
    RunCase2();
}

} // namespace
} // namespace cpp
} // namespace tutorial

int main(int argc, char **argv)
{
    LOG(INFO) << "Start tutorial_cpp_001_simple_template_main ... ";
    tutorial::cpp::Run();
    return 0;
}
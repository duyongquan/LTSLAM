#include "gtest/gtest.h"

namespace tutorial {
namespace cpp {
    
// true_type，false_type：代表类型（类/结构体类型）
// true,false：代表值
// 而bool既可以代表true也可以代表false。而true_type类型代表的就是true，false_type类型代表的就是false.

TEST(Tutorial, StdTrue)
{
    std::true_type a;
    std::false_type b;
    std::cout << a << std::endl;
    std::cout << b << std::endl;
}

} // namespace cpp
} // namespace tutorial


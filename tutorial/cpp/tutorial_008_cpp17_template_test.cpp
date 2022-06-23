#include "gtest/gtest.h"

#include <mutex>
#include <complex>
#include <vector>
#include <tuple>
#include <utility>

namespace tutorial {
namespace cpp {
    
// 类模板参数推导
TEST(Tutorial, template)
{
    // C++17 before
    std::complex<double> c{1.1, 2.0};
    std::mutex mutex;
    std::lock_guard<std::mutex> lock{mutex};

    // C++17 after
    std::complex c2{1.1, 2.0};
    std::lock_guard lock2 {mutex};

    // 可以让容器来推导元素类型
    std::vector v1{1, 2, 3};           // OK ： 推 导 出 std::vector<int>
    std::vector v2{"hello", "world"};  // OK ： 推 导 出 std::vector<const char*>

}

TEST(Tutorial, template1)
{
    // 只要能根据初始值推导出所有模板参数就可以使用类模板参数推导。推导过程支持所有方式的初始化（只要保证初始化是有效的）
    // 因为 std::complex 只需要一个参数就可以初始化并推导出模板参数
    std::complex c1{1.1, 2.2}; // 推 导 出 std::complex<double>
    std::complex c2(2.2, 3.3); // 推 导 出 std::complex<double>
    std::complex c3 = 3.3;     // 推 导 出 std::complex<double>
    std::complex c4 = {4.4};   // 推 导 出 std::complex<double>

    // 注意推导的过程中模板参数必须没有歧义
    // std::complex c5{5, 3.3}; // ERROR ： 尝 试 将 T 推 导 为 int 和 double
}

// 可变参数模板使用类模板参数推导
TEST(Tutorial, template2)
{
    std::tuple t{42, 'x', nullptr}; // 推导出类型 std::tuple<int, char, std::nullptr_t>。
}

template<typename T, int SZ>
class MyClass 
{
public:
  MyClass (T(&)[SZ]) 
  {
  // ...
  }
};

TEST(Tutorial, template3)
{
  tutorial::cpp::MyClass mc("hello"); // 推 导 出 T 为 const char ， SZ 为 6
}


// 默认以拷贝方式推导
TEST(Tutorial, template4)
{
    // 类 模 板 参 数 推 导 过 程 中 会 首 先 尝 试 以 拷 贝 的 方 式 初 始 化。
    std::vector v1{42}; // 一 个 元 素 的 vector<int>
    std::vector v2{v1}; // v2 也 是 一 个 std::vector<int>

    std::vector v5{v1}; // v5 也 是 vector<int>
    std::vector v3(v1); // v3 也 是 vector<int>
    std::vector v4 = {v1}; // v4 也 是 vector<int>
    auto v6 = std::vector{v1}; // v6 也 是 vector<int>

    // std::vector vv{v1, v2}; // vv 是 一 个 vector<vector<int>>
}

template<typename... Args>
auto make_vector(const Args&... elems) 
{
    return std::vector{elems...};
}

TEST(Tutorial, template5)
{
    std::vector<int> v{1, 2, 3};
    auto x1 = tutorial::cpp::make_vector(v, v); // vector<vector<int>>
    auto x2 = tutorial::cpp::make_vector(v); // vector<int> 还 是 vector<vector<int>>?
}

template<typename CB>
class CountCalls
{
private:
    CB callback; // 要调用的回调函数
    long calls = 0; // 调用的次数

public:
    CountCalls(CB cb) : callback(cb) 
    {
    }

    template<typename... Args>
    decltype(auto) operator() (Args&&... args) 
    {
      ++calls;
      return callback(std::forward<Args>(args)...);
    }

    long count() const 
    {
      return calls;
    }
};

// 推导 lambda 的类型
TEST(Tutorial, template6)
{
    tutorial::cpp::CountCalls sc{[](auto x, auto y) {
       return x > y; 
    }};

    std::vector v {8, 4, 7, 1, 2, 5, 0, 10};
    std::sort(v.begin(), v.end(),
    std::ref(sc));
    // 排 序 区 间
    // 排 序 准 则
    std::cout << "sorted with " << sc.count() << " calls\n";

    auto fo = std::for_each(v.begin(), v.end(), tutorial::cpp::CountCalls{[](auto i) {
        std::cout << "elem: " << i << '\n';
    }});
    std::cout << "output with " << fo.count() << " calls\n";


    // 如果计数器是原子的，你也可以使用并行算法:
    // std::sort(std::execution::par, v.begin(), v.end(), std::ref(sc));
}

} // namespace cpp
} // namespace tutorial


#include "gtest/gtest.h"
#include "glog/logging.h"

using namespace std;

namespace tutorial {
namespace cpp {


//递归终止函数
void print()
{
   cout << "empty" << endl;
}

//展开函数
template <class T, class ...Args>
void print(T head, Args... rest)
{
   cout << "parameter " << head << endl;
   print(rest...);
}

template<typename T>
T sum(T t)
{
    return t;
}

template<typename T, typename ... Types>
T sum(T first, Types ... rest)
{
    return first + sum<T>(rest...);
}

template <class T>
void printarg(T t)
{
   cout << t << endl;
}

template <class ...Args>
void expand(Args... args)
{
   int arr[] = {(printarg(args), 0)...};
}

template<class F, class... Args>void expand1(const F& f, Args&&...args) 
{
  //这里用到了完美转发，关于完美转发，读者可以参考笔者在上一期程序员中的文章《通过4行代码看右值引用》
  initializer_list<int>{(f(std::forward< Args>(args)),0)...};
}



//前向声明
template<typename... Args>
struct Sum;

//基本定义
template<typename First, typename... Rest>
struct Sum<First, Rest...>
{
    enum { value = Sum<First>::value + Sum<Rest...>::value };
};

//递归终止
template<typename Last>
struct Sum<Last>
{
    enum { value = sizeof (Last) };
};




TEST(Tutorial, VariadicTemplates)
{
    print(1,2,3,4);
    auto s = sum(1,2,3,4); // 10
    std::cout << "sum : " << s << std::endl;

    expand(1,2,3,4);
    expand1([](int i){cout<<i<<endl;}, 1,2,3);
}

} // namespace cpp
} // namespace tutorial

#include "gtest/gtest.h"


#include <mutex>
#include <complex>
#include <vector>
#include <tuple>
#include <utility>
#include <iostream>
#include <optional>

namespace tutorial {
namespace cpp {

template <typename T, typename ... Args>
auto Instance(Args&& ... args)
{
	return std::make_shared<T>(T(std::forward<Args>(args) ...));
}

// template <typename T, typename ... Args>
// T* Instance(Args&& ... args)
// {
// 	return new T(std::forward<Args>(args) ...);
// }

struct A
{
	A(int){
    std::cout << "A" << std::endl;
  }

  void print()
  {
    std::cout << "&&&&&&&&&&&" << std::endl;
  }
};

struct B
{
	B(int, double){
    std::cout << "B" << std::endl;
  }
};

TEST(Tutorial, demo01)
{
  	auto pa = Instance<A>(1);
    pa->print();

	  // B* pb = Instance<B>(1, 2);
}


TEST(Tutorial, demo02)
{
  std::optional<int> op ;
  if (op) {
    std::cout << *op << std::endl;
  }

  std::optional<int> op1 {2} ;
  if (op1) {
    std::cout << *op1 << std::endl;
  }
}



} // namespace cpp
} // namespace tutorial


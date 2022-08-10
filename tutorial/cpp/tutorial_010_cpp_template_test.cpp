#include "gtest/gtest.h"


#include <mutex>
#include <complex>
#include <vector>
#include <tuple>
#include <utility>
#include <iostream>
#include <functional>
#include <type_traits>
#include <list>

namespace tutorial {
namespace cpp {

struct one_tag {};
struct two_tag {};
struct three_tag {};

struct OneTest {
  using test_type = one_tag;
};


struct TwoTest {
  using test_type = two_tag;
};


struct ThreeTest {
  using test_type = three_tag;
};


class Test
{
public:

  template <typename T>
  void Run() {
    using Type = typename T::test_type;
    Run(Type());
  }

  void Run(const three_tag& test) {
    std::cout << "three_tag" << std::endl;
  }

  void Run(const two_tag& test) {
    std::cout << "two_tag" << std::endl;
  }

  void Run(const one_tag& test) {
    std::cout << "one_tag" << std::endl;
  }
};


struct Foo 
{
    Foo(int num) : num_(num) {}
    void print_add(int i) const { std::cout << num_+i << '\n'; }
    int num_;
};
 
void print_num(int i)
{
    std::cout << i << '\n';
}
 
struct PrintNum {
    void operator()(int i) const
    {
        std::cout << i << '\n';
    }
};

template<typename Ret = void>
struct CommCommand
{
private:
	std::function < Ret()> m_f;

public:
	//接受可调用对象的函数包装器
	template< class F, class... Args, class = typename std::enable_if<!std::is_member_function_pointer<F>::value>::type>
	void Wrap(F && f, Args && ... args)
	{
    std::cout << "#1" << std::endl;
		m_f = [&]{return f(args...); };
	}

	//接受常量成员函数的函数包装器
	template<class R, class C, class... DArgs, class P, class... Args>
	void Wrap(R(C::*f)(DArgs...) const, P && p, Args && ... args)
	{
    std::cout << "#2" << std::endl;
		m_f = [&, f]{return (*p.*f)(args...); };
	}

	//接受非常量成员函数的函数包装器 
	template<class R, class C, class... DArgs, class P, class... Args>
	void Wrap(R(C::*f)(DArgs...), P && p, Args && ... args)
	{
    std::cout << "#3" << std::endl;
		m_f = [&, f]{return (*p.*f)(args...); };
	}

	Ret Excecute()
	{
		return m_f();
	}
};


// Receiver类，知道如何实施与执行一个与请求相关的操作：
class Receiver 
{
public:
    void Action() 
    {
        std::cout << "Receiver" << std::endl;
    }
};

// Command类，用来声明执行操作的接口
class Command 
{
public:
    virtual void Excute() = 0;
    virtual void setReceiver(Receiver* r) = 0;
    virtual ~Command(){};
};

// ConcreteCommand类，绑定一个Receiver，调用其相应操作以实现Excute：
class ConcreteCommand : public Command 
{
private:
    Receiver* receiver;
public:
    void setReceiver(Receiver* r) 
    {
        receiver = r;
    }
    void Excute() {
        //cout << "ConcreteCommand" << endl;
        receiver->Action();
    }
};

// 要求该命令执行这个请求：
class Invoker 
{
private:
    std::list<Command* > commands;

public:
    void setCommand(Command* c) 
    {
        commands.push_back(c);
    }
    void Notify() 
    {
        for (auto c = commands.begin(); c != commands.end(); c++) {
            (*c)->Excute();
        }
    }
};

// 要求该命令执行这个请求：
template <typename R>
class Invoker2
{
private:
    std::list<CommCommand<R>> commands;

public:
    void setCommand(CommCommand<R>* c) 
    {
        commands.push_back(c);
    }

    void Notify() 
    {
        for (auto c = commands.begin(); c != commands.end(); c++) {
            (*c)->Excecute();
        }
    }
};


int add_one(int n)
{
  return n + 1;
}

}
}

TEST(Tutorial, demo01)
{
  tutorial::cpp::Test test;
  test.Run<tutorial::cpp::OneTest>();
}

TEST(Tutorial, demo02_invoke)
{
  // invoke a free function
  std::invoke(tutorial::cpp::print_num, -9);
 
  // invoke a lambda
  std::invoke([]() { tutorial::cpp::print_num(42); });

  // invoke a member function
  const tutorial::cpp::Foo foo(314159);
  std::invoke(&tutorial::cpp::Foo::print_add, foo, 1);

  // invoke (access) a data member
  std::cout << "num_: " << std::invoke(&tutorial::cpp::Foo::num_, foo) << '\n';

  // invoke a function object
  std::invoke(tutorial::cpp::PrintNum(), 18);
}


TEST(Tutorial, demo03_command_pattern)
{
  tutorial::cpp::Command* c = new tutorial::cpp::ConcreteCommand();
  tutorial::cpp::Receiver* r = new tutorial::cpp::Receiver();
  c->setReceiver(r);

  tutorial::cpp::Invoker i;
  i.setCommand(c);
  i.Notify();   // Receiver

  delete r;
  delete c;

}

TEST(Tutorial, demo04_command_pattern)
{
  tutorial::cpp::CommCommand<int> cmd;
  cmd.Wrap(tutorial::cpp::add_one, 1);
  auto result = cmd.Excecute();
  std::cout << "result: " << result << std::endl;
}


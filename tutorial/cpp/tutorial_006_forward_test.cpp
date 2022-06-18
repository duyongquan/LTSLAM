#include "gtest/gtest.h"
#include "glog/logging.h"

#include <string>
#include <utility>

namespace tutorial {
namespace cpp {

// std::move : 移除左值属性。
// std::forward : 完美转发。
// 假如我们封装了一个操作，主要是用来创建对象使用
// 1 可以接受不同类型的参数，然后构造一个对象的指针。
// 2 性能尽可能高。

class CData
{
public:
	CData() = delete;

	CData(const char* ch) : data(ch)
	{
		std::cout << "CData(const char* ch)" << std::endl;
	}

	CData(const std::string& str) : data(str) 
	{
		std::cout << "CData(const std::string& str)" << std::endl;
	}

	CData(std::string&& str) : data(str)
	{
		std::cout << "CData(std::string&& str)" << std::endl;
	}

	~CData()
	{
		std::cout << "~CData()" << std::endl;
	}

private:
	std::string data;
};

// 如果只要一个函数入口来创建对象，那么使用模板是不错的选择
// 这种办法虽然行得通，但是比较挫，因为每次调用Creator(T t)的时候，都需要拷贝内存，明显不满足高效的情况
// template<typename T>
// CData* Creator(T t)
// {
// 	return new CData(t);
// }

// // 上面说的值拷贝不能满足内容，那么我们使用引用就可以解决问题了吧？
// template<typename T>
// CData* Creator(T& t)
// {
// 	return new CData(t);
// }
// void Forward()
// {
// 	const char* value = "hello";
// 	std::string str1 = "hello";
// 	std::string str2 = " world";
// 	CData* p = Creator(value);
// 	//CData* p = Creator(str1);

// 	delete p;
// }


// void Forward()
// {
// 	const char* value = "hello";
// 	std::string str1 = "hello";
// 	std::string str2 = " world";
// 	//CData* p = Creator(value);
// 	CData* p = Creator(str1);
// 	//CData* p = Creator(str1 + str2);

// 	delete p;
// }


// 所谓的完美转发，是指std::forward会将输入的参数原封不动地传递到下一个函数中，这个“原封不动”指的是，
// 如果输入的参数是左值，那么传递给下一个函数的参数的也是左值；如果输入的参数是右值，那么传递给下一个函数的参数的也是右值。

TEST(Tutorial, forward)
{
    
}

// std::move并不能移动任何东西，它唯一的功能是将一个左值强制转化为右值引用，
// 继而可以通过右值引用使用该值，以用于移动语义。从实现上讲，
// std::move基本等同于一个类型转换：static_cast<T&&>(lvalue);
// 1 C++ 标准库使用比如vector::push_back 等这类函数时,会对参数的对象进行复制,连数据也会复制.这就会造成对象内存的额外创建, 
//   本来原意是想把参数push_back进去就行了,通过std::move，可以避免不必要的拷贝操作。
// 2 std::move是将对象的状态或者所有权从一个对象转移到另一个对象，只是转移，
//   没有内存的搬迁或者内存拷贝所以可以提高利用效率,改善性能.。
// 3 对指针类型的标准库对象并不需要这么做.
TEST(Tutorial, move)
{
    std::string str = "Hello";
    std::vector<std::string> v;

    //调用常规的拷贝构造函数，新建字符数组，拷贝数据
    v.push_back(str);
    std::cout << "After copy, str is \"" << str << "\"\n";

    //调用移动构造函数，掏空str，掏空后，最好不要使用str
    v.push_back(std::move(str));
    std::cout << "After move, str is \"" << str << "\"\n";
    std::cout << "The contents of the vector are \"" << v[0]
                                         << "\", \"" << v[1] << "\"\n";
}

void f(int&) { std::cout << 1; }
void f(const int&) { std::cout << 2; }
void f(int&&) { std::cout << 3; }

// 用多个重载转发给对应版本比较繁琐
void g(int& x)
{
  f(x);
}

void g(const int& x)
{
  f(x);
}

void g(int&& x)
{
  f(std::move(x));
}

// 同样可以用一个模板来替代上述功能
template<typename T>
void h(T&& x)
{
  f(std::forward<T>(x)); // 注意std::forward的模板参数是T
}

TEST(Tutorial, template_forward)
{
     int a = 1;
    const int b = 1;

    g(a); h(a); // 11
    g(b); h(b); // 22
    g(std::move(a)); h(std::move(a)); // 33
    g(1); h(1); // 33
}

} // namespace cpp
} // namespace tutorial


.. _chapter-c++_tutorial:

============
C++ Tutorial
============

C++ 发展历程
====================

* C++ 编程语言的历史可以追溯到 **1979 年**，当时 Bjarne Stroustrup 为博士学位论文进行了一些开发。在 Stroustrup 可以使用的所有语言中，有一种被称为 Simula 的语言，顾名思义，它可能是一种主要为仿真而设计的语言。Simula 67 语言是 Stroustrup 使用的变体，被认为是支持面向对象编程范例的主要语言。Stroustrup 发现这种范例对包装开发很有帮助。但是，Simula 语言对于实践和实际使用而言太慢了。
    随后不久，Bjarne Stroustrup 希望通过支持面向对象范例来增强 C。他深入研究了 Smalltalk 的 OO 实现，以获取有关实现的想法。但是他不愿意为此放弃性能，因此他开始从事 “C with Classes (带有类的 C）” 的工作，希望 C++ 代码运行时应具有与 C 代码相似（或更好）的性能。

* **1983 年**，语言的名称从 “带有类的 C” 更改为 C++。C 语言中的 ++ 运算符是用于递增变量的运算符，它使您可以深入了解 Stroustrup 如何看待该语言。在此期间添加了许多新功能，其中最引人注目的是虚函数，函数重载，带有＆符号的引用，const 关键字和使用两个正斜杠的单行注释。

* **1985 年**，Stroustrup 出版了名为*“C++ 编程语言” 的书籍*。同年，C++ 被实现为商业产品。该语言尚未正式标准化，因此使该书成为非常重要的参考。该语言在 1989 年再次进行了更新，以包括受保护的成员和静态成员，以及从多个类的继承。

* **1990 年**，发行了*《带注释的 C++ 参考手册*》。同年，Borland 的 Turbo C++ 编译器将作为商业产品发布。Turbo C++ 添加了许多其他库，这些库会对 C++ 的开发产生相当大的影响。尽管 Turbo C++ 的最后一个稳定版本是 2006 年，但该编译器仍被广泛使用。

* **1998 年**，C++ 标准委员会发布了第一个 C++ ISO / IEC 14882：1998 国际标准，其非正式名称为 C++ 98。据说*《带注释的 C++ 参考手册*》对标准的制定产生了很大的影响。还包括标准模板库，该模板库于 1979 年开始概念开发。2003 年，该委员会对 1998 年标准所报告的多个问题做出了回应，并对其进行了相应的修订。更改的语言称为 C++ 03。

* **2005 年**，C++ 标准委员会发布了一份技术报告（称为 TR1），详细介绍了他们计划添加到最新 C++ 标准中的各种功能。新标准被非正式地称为 C++ 0x，因为它有望在第一个十年结束之前的某个时间发布。具有讽刺意味的是，新标准要到 2011 年年中才会发布。直到那时为止，已经发布了几份技术报告，并且一些编译器开始为新功能添加实验性支持。

* **2011 年中**，新的 C++ 标准（称为 C++ 11）完成。Boost 库项目对新标准产生了重大影响，其中一些新模块直接来自相应的 Boost 库。一些新功能包括正则表达式支持，全面的随机化库，新的 C++ 时间库，原子支持，标准线程库 ，一种新的 for 循环语法，提供的功能类似于某些其他语言中的 foreach 循环，auto 关键字，新的容器类，对联合和数组初始化列表以及可变参数模板的更好支持。

* **2014 年**，C++ 14（也称为 C++ 1y）作为 C++11 的一个小扩展发布，主要功能是错误修复和小的改进，国际标准投票程序草案于 2014 年 8 月中完成，加强 lambda 函数，constexpr 和类型推导特性。

* **2017 年**，发布 C17 标准，C17 提供了很多东西。增强了核心语言和库。

* **2020 年**，发布 C++20 标准，推出了很多重量级功能，其中比较重要的有：

    - Concepts：概念改变了我们思考和编程模板的方式。它们是模板参数的语义类别。它们使您可以直接在类型系统中表达您的意图。如果出了什么问题，您会收到清晰的错误消息。
    - Ranges library：新的 ranges 库使它可以直接在容器上执行算法，用管道符号组成算法，并将其应用于无限数据流。
    - Coroutines：由于协程，C++ 中的异步编程成为主流。协程是协作任务，事件循环，无限数据流或管道的基础。
    - Modules：模块克服了头文件的限制。头文件和源文件的分离变得和预处理器一样过时了。最后，我们有更快的构建时间和更轻松的构建软件包的方法。
    - Concurrency：Atomic Smart Pointers,Joining & Cancellable Threads,The C20 Synchronization Library，增强了 C++ 并发编程能力；
    

书籍推荐
====================


* `深入理解C++11：C++11新特性解析与应用 <https://www.aliyundrive.com/s/LKc1X2mL9G9>`_

* `C++入门经典（第10版） <https://book.douban.com/subject/30247747/>`_

* `C++ Primer Plus 第6版 中文版(异步图书出品) <https://www.epubit.com/bookDetails?id=UB7209840d845c9>`_

* `清华计算机图书译丛：精通C++（第9版） <https://item.jd.com/12432130.html>`_

* `C++高级编程(第4版) <http://www.tup.tsinghua.edu.cn/booksCenter/book_07894801.html>`_

* `C++游戏编程入门（第4版）(异步图书出品) <https://item.jd.com/13265350.html>`_

* `STL源码剖析 <https://book.douban.com/subject/1110934/>`_

* `C++程序设计:原理与实践(基础篇)(原书第2版) <https://book.douban.com/subject/27023080/>`_

* `Accelerated C++中文版 <https://book.douban.com/subject/2280545//>`_

* `C++编程思想(两卷合订本) <https://book.douban.com/subject/6558198/>`_

* `中文版Effective STL:50条有效使用STL的经验 <https://book.douban.com/subject/1792179/>`_

* `C++编程剖析:问题、方案和设计准则 <https://book.douban.com/subject/5367371/>`_

* `C++ Templates中文版 <https://book.douban.com/subject/1144020/>`_

* `C++设计新思维 <https://book.douban.com/subject/1103566/>`_

* `C++模板元编程 <https://book.douban.com/subject/4136223/>`_

* `C++并发编程实战 <https://book.douban.com/subject/26386925/>`_

* `C++程序设计语言(第1-3部分)(原书第4版) <https://book.douban.com/subject/26857943/>`_

* `C++标准库(第2版) <https://book.douban.com/subject/26419721/>`_

* `Essential C++ <https://book.douban.com/subject/24868427/>`_

* `C++ 语言的设计与演化 <https://book.douban.com/subject/1096216/>`_

* `深度探索C++ 对象模型 <https://book.douban.com/subject/1091086/>`_

* `泛型编程与STL <https://book.douban.com/subject/1241423/>`_

文章推荐
====================

* `每个c++开发人员都应该使用的10个c++11特性 <https://github.com/0voice/cpp_new_features/blob/main/%E6%AF%8F%E4%B8%AAc%2B%2B%E5%BC%80%E5%8F%91%E4%BA%BA%E5%91%98%E9%83%BD%E5%BA%94%E8%AF%A5%E4%BD%BF%E7%94%A8%E7%9A%8410%E4%B8%AAc%2B%2B%2011%E7%89%B9%E6%80%A7.md>`_

* `在c++项目中你必须真正使用的15个c++11特性 <https://github.com/0voice/cpp_new_features/blob/main/%E5%9C%A8c%2B%2B%E9%A1%B9%E7%9B%AE%E4%B8%AD%E4%BD%A0%E5%BF%85%E9%A1%BB%E7%9C%9F%E6%AD%A3%E4%BD%BF%E7%94%A8%E7%9A%8415%E4%B8%AAc%2B%2B%E7%89%B9%E6%80%A7.md>`_

* `如何在 C++11 中使用 Lambda 表达式 <https://github.com/0voice/cpp_new_features/blob/main/%E5%A6%82%E4%BD%95%E5%9C%A8%20C%2B%2B11%20%E4%B8%AD%E4%BD%BF%E7%94%A8%20Lambda%20%E8%A1%A8%E8%BE%BE%E5%BC%8F.md>`_

* `深入理解C++11 <https://github.com/0voice/cpp_new_features/blob/main/%E6%B7%B1%E5%85%A5%E7%90%86%E8%A7%A3C%2B%2B11.md>`_

* `吐血整理：C++11新特性 <https://github.com/0voice/cpp_new_features/blob/main/%E5%90%90%E8%A1%80%E6%95%B4%E7%90%86%EF%BC%9AC%2B%2B11%E6%96%B0%E7%89%B9%E6%80%A7.md>`_

* `C++11新特性之auto和decltype知识点 <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11%E6%96%B0%E7%89%B9%E6%80%A7%E4%B9%8Bauto%E5%92%8Cdecltype%E7%9F%A5%E8%AF%86%E7%82%B9>`_

* `C++11新特性之左值引用、右值引用、移动语义、完美转发 <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11%E6%96%B0%E7%89%B9%E6%80%A7%E4%B9%8B%E5%B7%A6%E5%80%BC%E5%BC%95%E7%94%A8%E5%8F%B3%E5%80%BC%E5%BC%95%E7%94%A8%E7%A7%BB%E5%8A%A8%E8%AF%AD%E4%B9%89%E5%AE%8C%E7%BE%8E%E8%BD%AC%E5%8F%91>`_

* `C++11新特性之列表初始化 <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11%E6%96%B0%E7%89%B9%E6%80%A7%E4%B9%8B%E5%88%97%E8%A1%A8%E5%88%9D%E5%A7%8B%E5%8C%96>`_

* `C++11新特性std::function和lambda表达式 <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11%E6%96%B0%E7%89%B9%E6%80%A7stdfunction%E5%92%8Clambda%E8%A1%A8%E8%BE%BE%E5%BC%8F>`_

* `C++11新特性之模板改进 <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11%E6%96%B0%E7%89%B9%E6%80%A7%E4%B9%8B%E6%A8%A1%E6%9D%BF%E6%94%B9%E8%BF%9B>`_

* `C++11新特性之线程相关知识点 <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11%E6%96%B0%E7%89%B9%E6%80%A7%E4%B9%8B%E7%BA%BF%E7%A8%8B%E7%9B%B8%E5%85%B3%E7%9F%A5%E8%AF%86%E7%82%B9>`_

* `C++11新特性之异步操作-async <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11-%E7%9A%84%E5%BC%82%E6%AD%A5%E6%93%8D%E4%BD%9C-async>`_

* `C++11新特性之智能指针 <https://github.com/0voice/cpp_new_features/blob/main/%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0%EF%BC%9AC++%2011%E6%96%B0%E7%89%B9%E6%80%A7.md#c11%E6%96%B0%E7%89%B9%E6%80%A7%E4%B9%8B%E6%99%BA%E8%83%BD%E6%8C%87%E9%92%88>`_

* `C++11常用新特性（一） <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B11%E5%B8%B8%E7%94%A8%E6%96%B0%E7%89%B9%E6%80%A7%EF%BC%88%E4%B8%80%EF%BC%89.md>`_

* `C++11常用新特性（二） <https://github.com/0voice/cpp_new_features/blob/main/C++11%E5%B8%B8%E7%94%A8%E6%96%B0%E7%89%B9%E6%80%A7%EF%BC%88%E4%BA%8C%EF%BC%89.md>`_

* `C++14新特性浅谈 <https://github.com/0voice/cpp_new_features/blob/main/%E3%80%8CNotes%E3%80%8DC%2B%2B14%E6%96%B0%E7%89%B9%E6%80%A7%E6%B5%85%E8%B0%88.md>`_

* `C++14新特性的所有知识点全在这儿啦 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B14%E6%96%B0%E7%89%B9%E6%80%A7%E7%9A%84%E6%89%80%E6%9C%89%E7%9F%A5%E8%AF%86%E7%82%B9%E5%85%A8%E5%9C%A8%E8%BF%99%E5%84%BF%E5%95%A6%EF%BC%81.md>`_

* `总结归纳：C++17新特性 <https://github.com/0voice/cpp_new_features/blob/main/%E6%80%BB%E7%BB%93%E5%BD%92%E7%BA%B3%EF%BC%9AC%2B%2B17%E6%96%B0%E7%89%B9%E6%80%A7.md>`_

* `C++ 20语言特性 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%2020%E8%AF%AD%E8%A8%80%E7%89%B9%E6%80%A7.md>`_

C++ 入门教程（41课时） - 阿里云大学
===========================================

* `C++ 简介 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E7%AE%80%E4%BB%8B>`_

* `C++ 环境设置 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E7%8E%AF%E5%A2%83%E8%AE%BE%E7%BD%AE>`_

* `C++ 基本语法 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%9F%BA%E6%9C%AC%E8%AF%AD%E6%B3%95>`_

* `C++ 注释 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%B3%A8%E9%87%8A>`_

* `C++ 数据类型 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%95%B0%E6%8D%AE%E7%B1%BB%E5%9E%8B>`_

* `C++ 变量类型 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%8F%98%E9%87%8F%E7%B1%BB%E5%9E%8B>`_

* `C++ 变量作用域 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%8F%98%E9%87%8F%E4%BD%9C%E7%94%A8%E5%9F%9F>`_

* `C++ 常量 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%B8%B8%E9%87%8F>`_

* `C++ 修饰符类型 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E4%BF%AE%E9%A5%B0%E7%AC%A6%E7%B1%BB%E5%9E%8B>`_

* `C++ 存储类 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%AD%98%E5%82%A8%E7%B1%BB>`_

* `C++ 运算符 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E8%BF%90%E7%AE%97%E7%AC%A6>`_

* `C++ 循环 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%BE%AA%E7%8E%AF>`_

* `C++ 判断 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%88%A4%E6%96%AD>`_

* `C++ 函数 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%87%BD%E6%95%B0>`_

* `C++ 数字 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%95%B0%E5%AD%97>`_

* `C++ 数组 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%95%B0%E7%BB%84>`_

* `C++ 字符串 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%AD%97%E7%AC%A6%E4%B8%B2>`_

* `C++ 指针 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%8C%87%E9%92%88>`_

* `C++ 引用 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%BC%95%E7%94%A8>`_

* `C++ 日期 & 时间 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%97%A5%E6%9C%9F--%E6%97%B6%E9%97%B4>`_

* `C++ 基本的输入输出 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%9F%BA%E6%9C%AC%E7%9A%84%E8%BE%93%E5%85%A5%E8%BE%93%E5%87%BA>`_

* `C++ 数据结构 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%95%B0%E6%8D%AE%E7%BB%93%E6%9E%84>`_

* `C++ 类 & 对象 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E7%B1%BB--%E5%AF%B9%E8%B1%A1>`_

* `C++ 继承 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E7%BB%A7%E6%89%BF>`_

* `C++ 重载运算符和重载函数 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E9%87%8D%E8%BD%BD%E8%BF%90%E7%AE%97%E7%AC%A6%E5%92%8C%E9%87%8D%E8%BD%BD%E5%87%BD%E6%95%B0>`_

* `C++ 多态 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%A4%9A%E6%80%81>`_

* `C++ 数据抽象 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%95%B0%E6%8D%AE%E6%8A%BD%E8%B1%A1>`_

* `C++ 数据封装 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%95%B0%E6%8D%AE%E5%B0%81%E8%A3%85>`_

* `C++ 接口（抽象类） <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%8E%A5%E5%8F%A3%E6%8A%BD%E8%B1%A1%E7%B1%BB>`_

* `C++ 文件和流 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%96%87%E4%BB%B6%E5%92%8C%E6%B5%81>`_

* `C++ 异常处理 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%BC%82%E5%B8%B8%E5%A4%84%E7%90%86>`_

* `C++ 动态内存 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%8A%A8%E6%80%81%E5%86%85%E5%AD%98>`_

* `C++ 命名空间 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%91%BD%E5%90%8D%E7%A9%BA%E9%97%B4>`_

* `C++ 模板 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%A8%A1%E6%9D%BF>`_

* `C++ 预处理器 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E9%A2%84%E5%A4%84%E7%90%86%E5%99%A8>`_

* `C++ 信号处理 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E4%BF%A1%E5%8F%B7%E5%A4%84%E7%90%86>`_

* `C++ 多线程 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E5%A4%9A%E7%BA%BF%E7%A8%8B>`_

* `C++ Web 编程 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-web-%E7%BC%96%E7%A8%8B>`_

* `C++ STL 教程 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-stl-%E6%95%99%E7%A8%8B>`_

* `C++ 标准库 <https://github.com/0voice/cpp_new_features/blob/main/C%2B%2B%20%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%EF%BC%8841%E8%AF%BE%E6%97%B6%EF%BC%89%20-%20%E9%98%BF%E9%87%8C%E4%BA%91%E5%A4%A7%E5%AD%A6.md#c-%E6%A0%87%E5%87%86%E5%BA%93>`_


推荐学习的CPP网站
====================

* `cppreference <https://zh.cppreference.com/w/%E9%A6%96%E9%A1%B5>`_

* `purecpp论坛 <http://purecpp.org/>`_

* `Google 开源项目风格指南——中文版 <https://zh-google-styleguide.readthedocs.io/en/latest/contents/>`_

* `C++ Templates - The Complete Guide <http://www.tmplbook.com./>`_

* `cpp-cheat-sheets <https://hackingcpp.com/cpp/cheat_sheets.html>`_

* `侯捷 C++ 系列 <https://github.com/ZachL1/Bilibili-plus>`_


github开源项目推荐
====================

* `cpp_new_features <https://github.com/0voice/cpp_new_features>`_

.. figure:: cpp_new_features.png
   :align: center

* `awesome-cpp <https://github.com/fffaraz/awesome-cpp>`_

* `CPlusPlusThings <https://github.com/Light-City/CPlusPlusThings>`_

* `CppTemplateTutorial <https://github.com/wuye9036/CppTemplateTutorial>`_

* `modern-cpp-tutorial <https://github.com/changkun/modern-cpp-tutorial>`_

* `advanced-cplusplus <https://github.com/caveofprogramming/advanced-cplusplus>`_

工程中使用到的C++技能
=========================
.. toctree::
   :maxdepth: 2

   cpp11_14_17/cpp11_14_17
   smart_pointer/smart_pointer
   template/template
   threading/threading
   stl/stl
   design_patterns/design_patterns
   3rdparty/3rdparty

Linux工具快速教程
==========================

* `Linux工具快速教程 <https://linuxtools-rst.readthedocs.io/zh_CN/latest/index.html>`_
* `GDB 调试利器 <https://linuxtools-rst.readthedocs.io/zh_CN/latest/tool/gdb.html>`_

框架
==========================

* `Apache C++ Standard Library <http://stdcxx.apache.org/>`_  : 是一系列算法，容器，迭代器和其他基本组件的集合

* `ASL <http://stlab.adobe.com/>`_  : Adobe源代码库提供了同行的评审和可移植的C++源代码库。

* `Boost <https://github.com/boostorg>`_  : 大量通用C++库的集合。

* `BDE <https://github.com/bloomberg/bde>`_  : 来自于彭博资讯实验室的开发环境。

* `Cinder <https://libcinder.org/>`_  : 提供专业品质创造性编码的开源开发社区。

* `Bxxomfort <http://ryan.gulix.cl/fossil.cgi/cxxomfort/index>`_  : 轻量级的，只包含头文件的库，将C++ 11的一些新特性移植到C++03中。

* `Dlib <http://dlib.net/>`_  : 使用契约式编程和现代C++科技设计的通用的跨平台的C++库。

* `EASTL <https://github.com/paulhodge/EASTL>`_  : EA-STL公共部分

* `ffead-cpp <https://github.com/sumeetchhetri/ffead-cpp>`_  : 企业应用程序开发框架

* `Folly <https://github.com/facebook/folly>`_  : 由Facebook开发和使用的开源C++库。

* `JUCE <https://github.com/WeAreROLI/JUCE>`_  : 包罗万象的C++类库，用于开发跨平台软件

* `libphenom <https://github.com/facebookarchive/libphenom>`_  : 用于构建高性能和高度可扩展性系统的事件框架。

* `LibSourcey <https://github.com/sourcey/libsourcey>`_  : 用于实时的视频流和高性能网络应用程序的C++11 evented IO

* `Loki <http://loki-lib.sourceforge.net/>`_  : C++库的设计，包括常见的设计模式和习语的实现。

* `MiLi <https://code.google.com/p/mili/>`_  : 只含头文件的小型C++库

* `openFrameworks <https://openframeworks.cc/>`_  : 开发C++工具包，用于创意性编码。

* `Qt <https://www.qt.io/developers/>`_  : 跨平台的应用程序和用户界面框架

* `Reason <http://code.google.com/p/reason/>`_  : 跨平台的框架，使开发者能够更容易地使用Java，.Net和Python，同时也满足了他们对C++性能和优势的需求。

* `ROOT <https://root.cern.ch/>`_  : 具备所有功能的一系列面向对象的框架，能够非常高效地处理和分析大量的数据，为欧洲原子能研究机构所用。

* `STLport <http://www.stlport.org/>`_  : 是STL具有代表性的版本

* `STXXL <http://stxxl.sourceforge.net/>`_  : 用于额外的大型数据集的标准模板库。

* `Ultimate++ <https://www.ultimatepp.org/>`_  : C++跨平台快速应用程序开发框架

* `Windows Template Library <https://sourceforge.net/projects/wtl/>`_  : 用于开发Windows应用程序和UI组件的C++库

* `Yomm11 <https://github.com/jll63/yomm11>`_  : C++11的开放multi-methods.

异步事件循环
==========================

* `Boost.Asio <https://think-async.com/Asio/>`_  : 用于网络和底层I/O编程的跨平台的C++库。

* `libev <http://libev.schmorp.de/>`_  : 功能齐全，高性能的时间循环，轻微地仿效libevent，但是不再像libevent一样有局限性，也修复了它的一些bug。

* `libevent <http://libevent.org/>`_  : 事件通知库

* `libuv <https://github.com/joyent/libuv>`_  : 跨平台异步I/O。

* `libco <https://github.com/Tencent/libco>`_  : 协程，微信支持8亿用户同时在线的底层IO库。功能强大

* `libgo <https://github.com/yyzybb537/libgo>`_  : golang风格的并发框架，C++11实现协程库

网络库
==========================

* `ACE <https://github.com/cflowe/ACE>`_  : C++面向对象网络变成工具包
* `Casablanca <https://archive.codeplex.com/?p=casablanca>`_  : C++ REST SDK
* `cpp-netlib <https://cpp-netlib.org/>`_  : 高级网络编程的开源库集合
* `libCurl <https://curl.haxx.se/libcurl/>`_  : 多协议文件传输库
* `Mongoose <https://github.com/cesanta/mongoose>`_  : 非常轻量级的网络服务器
* `Muduo <https://github.com/chenshuo/muduo>`_  : 用于Linux多线程服务器的C++非阻塞网络库
* `net_skeleton <https://github.com/cesanta/fossa>`_  : C/C++的TCP 客户端/服务器库
* `POCO <https://github.com/pocoproject/poco>`_  : 用于构建网络和基于互联网应用程序的C++类库，可以运行在桌面，服务器，移动和嵌入式系统。
* `RakNet <https://github.com/facebookarchive/RakNet>`_  : 为游戏开发人员提供的跨平台的开源C++网络引擎。
* `Tufao <https://github.com/vinipsmaker/tufao>`_  : 用于Qt之上的C++构建的异步Web框架。
* `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_  : 基于C++/Boost Aiso的websocket 客户端/服务器库
* `ZeroMQ <http://zeromq.org/>`_  : 高速，模块化的异步通信库

TCP/IP协议栈
==========================

* `f-stack <https://github.com/f-stack/f-stack>`_  : 腾讯开源的协议栈，基于DPDK的高性能用户态协议栈。
* `NtyTcp <https://github.com/wangbojing/NtyTcp>`_  : 单线程的协议栈的，基于netmap,DPDK,rawSocket的实现。
* `LWIP <http://savannah.nongnu.org/projects/lwip/>`_  : 针对 RAM 平台的精简版的 TCP/IP 协议栈实现。
* `mTCP <https://github.com/mtcp-stack/mtcp>`_  : 针对多核系统的高可扩展性的用户空间 TCP/IP 协议栈。
* `4.4BSD <https://www.freebsd.org/zh_CN/copyright/license.html>`_  : * nix的协议栈是源于4.4BSD的。

WEB应用框架
==========================

* `Nginx <http://nginx.org/>`_  : 一个高性能的HTTP和反向代理web服务器，同时也提供了IMAP/POP3/SMTP服务。
* `Lighttpd <http://www.lighttpd.net/>`_  : 一款开源 Web 服务器软件，安全快速,符合行业标准,适配性强并且针对高配置环境进行了优化。
* `Libmicrohttpd <http://www.gnu.org/software/libmicrohttpd/>`_  : GNU软件下的简单c库的Web服务器。API简单，快速。
* `shttpd <http://shttpd.sourceforge.net/>`_  : 基于Mongoose的Web服务器框架。
* `CivetWeb <https://github.com/bel2125/civetweb>`_  : 提供易于使用，强大的，C/C++嵌入式Web服务器，带有可选的CGI，SSL和Lua支持。
* `CppCMS <http://cppcms.com/wikipp/en/page/main>`_  : 免费高性能的Web开发框架（不是 CMS）.
* `Crow <https://github.com/ipkn/crow>`_  : 一个C++微型web框架（灵感来自于Python Flask）
* `Kore <https://kore.io/>`_  : 使用C语言开发的用于web应用程序的超快速和灵活的web服务器/框架。
* `libOnion <https://www.coralbits.com/libonion/>`_  : 轻量级的库，帮助你使用C编程语言创建web服务器。
* `QDjango <https://github.com/jlaine/qdjango/>`_  : 使用C++编写的，基于Qt库的web框架，试图效仿Django API，因此得此名。
* `Wt <https://www.webtoolkit.eu/wt>`_  : 开发Web应用的C++库。

标准库，算法与函数
==========================

* `C++ Standard Library <http://en.wikipedia.org/wiki/C%2B%2B_Standard_Library>`_  : 是一系列类和函数的集合，使用核心语言编写，也是C++ISO自身标准的一部分。
* `Standard Template Library <https://en.wikipedia.org/wiki/Standard_Template_Library>`_  : 标准模板库, STL
* `ISO C++ Standards Committee <https://github.com/cplusplus>`_  : C++标准委员会

音频库
==========================

* `FMOD <https://www.fmod.com/>`_  : 易于使用的跨平台的音频引擎和音频内容的游戏创作工具。
* `Maximilian <https://github.com/micknoise/Maximilian>`_  : C++音频和音乐数字信号处理库
* `OpenAL <http://www.openal.org/>`_  : 开源音频库—跨平台的音频API
* `Opus <http://opus-codec.org/>`_  : 一个完全开放的，免版税的，高度通用的音频编解码器
* `Speex <https://www.speex.org/>`_  : 免费编解码器，为Opus所废弃
* `Tonic <https://github.com/TonicAudio/Tonic>`_  : C++易用和高效的音频合成
* `Vorbis <http://xiph.org/vorbis/>`_  : Ogg Vorbis是一种完全开放的，非专有的，免版税的通用压缩音频格式。

生态学
==========================

* `lisequence <http://molpopgen.github.io/libsequence/>`_  : 用于表示和分析群体遗传学数据的C++库。
* `SeqAn <http://www.seqan.de/>`_  : 专注于生物数据序列分析的算法和数据结构。
* `Vcflib <https://github.com/vcflib/vcflib>`_  : 用于解析和处理VCF文件的C++库
* `Wham <https://github.com/zeeev/wham>`_  : 直接把联想测试应用到BAM文件的基因结构变异。

压缩
==========================

* `bzip2 <http://www.bzip.org/>`_  : 一个完全免费，免费专利和高质量的数据压缩
* `doboz <https://bitbucket.org/attila_afra/doboz/src>`_  : 能够快速解压缩的压缩库
* `PhysicsFS <https://icculus.org/physfs/>`_  : 对各种归档提供抽象访问的库，主要用于视频游戏，设计灵感部分来自于Quake3的文件子系统。
* `KArchive <https://projects.kde.org/projects/frameworks/karchive>`_  : 用于创建，读写和操作文件档案（例如zip和 tar）的库，它通过QIODevice的一系列子类，使用gzip格式，提供了透明的压缩和解压缩的数据。
* `LZ4 <https://code.google.com/p/lz4/>`_  : 非常快速的压缩算法
* `LZHAM <https://code.google.com/p/lzham/>`_  : 无损压缩数据库，压缩比率跟LZMA接近，但是解压缩速度却要快得多。
* `LZMA <http://www.7-zip.org/sdk.html>`_  : 7z格式默认和通用的压缩方法。
* `LZMAT <http://www.matcode.com/lzmat.htm>`_  : 及其快速的实时无损数据压缩库
* `Minizip <https://code.google.com/p/miniz/>`_  : Zlib最新bug修复，支持PKWARE磁盘跨越，AES加密和IO缓冲。
* `Snappy <https://code.google.com/p/snappy/>`_  : 快速压缩和解压缩
* `ZLib <http://zlib.net/>`_  : 非常紧凑的数据流压缩库
* `ZZIPlib <http://zziplib.sourceforge.net/>`_  : 提供ZIP归档的读权限。

并发性
==========================

* `Boost.Compute <https://github.com/boostorg/compute>`_  : 用于OpenCL的C++GPU计算库
* `Bolt <https://github.com/HSA-Libraries/Bolt>`_  :  针对GPU进行优化的C++模板库
* `C++React <https://github.com/schlangster/cpp.react>`_  : 用于C++11的反应性编程库
* `Intel TBB <https://www.threadingbuildingblocks.org/>`_  : Intel线程构件块
* `Libclsph <https://github.com/libclsph/libclsph>`_  : 基于OpenCL的GPU加速SPH流体仿真库
* `OpenCL <https://www.khronos.org/opencl/>`_  : 并行编程的异构系统的开放标准
* `OpenMP <https://www.openmp.org/>`_  : OpenMP API
* `Thrust <http://thrust.github.io/>`_  : 类似于C++标准模板库的并行算法库
* `HPX <https://github.com/STEllAR-GROUP/hpx/>`_  : 用于任何规模的并行和分布式应用程序的通用C++运行时系统
* `VexCL <https://github.com/ddemidov/vexcl>`_  : 用于OpenCL/CUDA 的C++向量表达式模板库。

密码学
==========================

* `Bcrypt <http://bcrypt.sourceforge.net/>`_  : 一个跨平台的文件加密工具，加密文件可以移植到所有可支持的操作系统和处理器中。
* `BeeCrypt <https://github.com/klchang/beecrypt>`_  : 快速的加密图形库，功能强大，接口方便。
* `Botan <https://botan.randombit.net/>`_  : C++加密库
* `Crypto++ <https://www.cryptopp.com/>`_  : 一个有关加密方案的免费的C++库
* `GnuPG <https://www.gnupg.org/>`_  : OpenPGP标准的完整实现
* `GnuTLS <https://www.gnutls.org/>`_  : 实现了SSL，TLS和DTLS协议的安全通信库
* `Libgcrypt <https://gnupg.org/related_software/libgcrypt/>`_  : 基于GnuPG的加密图形库。
* `Libmcrypt <https://github.com/winlibs/libmcrypt>`_  : 线程安全，提供统一的API。
* `LibreSSL <http://www.libressl.org/>`_  : 免费的SSL/TLS协议，属于2014 OpenSSL的一个分支
* `LibTomCrypt <https://github.com/libtom/libtomcrypt>`_  : 一个非常全面的，模块化的，可移植的加密工具
* `libsodium <https://github.com/jedisct1/libsodium>`_  : 基于NaCI的加密库，固执己见，容易使用
* `Nettle <http://www.lysator.liu.se/~nisse/nettle/>`_  : 底层的加密库
* `OpenSSL <https://www.openssl.org/>`_  : 一个强大的，商用的，功能齐全的，开放源代码的加密库。

数据库
==========================

* `hiberlite <https://github.com/paulftw/hiberlite>`_  : 用于Sqlite3的C++对象关系映射
* `LevelDB <https://github.com/google/leveldb>`_  : 快速键值存储库
* `LMDB <https://symas.com/lmdb/technical/>`_  : 符合数据库四大基本元素的嵌入键值存储
* `MySQL++ <https://tangentsoft.com/mysqlpp/home>`_  : 封装了MySql的C API的C++ 包装器
* `RocksDB <https://github.com/facebook/rocksdb>`_  : 来自Facebook的嵌入键值的快速存储
* `SQLite <https://www.sqlite.org/index.html>`_  : 一个完全嵌入式的，功能齐全的关系数据库，只有几百KB，可以正确包含到你的项目中。
* `MongoDB <https://www.mongodb.com/>`_  : 一个基于分布式文件存储的数据库

调试
==========================

* `Boost.Test <https://www.boost.org/doc/libs/master/libs/test/doc/html/index.html>`_  : Boost测试库
* `Catch <https://github.com/catchorg/Catch2>`_  : 一个很时尚的，C++原生的框架，只包含头文件，用于单元测试，测试驱动开发和行为驱动开发。
* `CppUnit <https://www.freedesktop.org/wiki/Software/cppunit/>`_  : 由JUnit移植过来的C++测试框架
* `GoogleTest <http://code.google.com/p/googletest/>`_  : 谷歌C++测试框架
* `ig-debugheap <https://github.com/deplinenoise/ig-debugheap>`_  : 用于跟踪内存错误的多平台调试堆
* `MemTrack <http://www.almostinfinite.com/memtrack.html>`_  : 用于C++跟踪内存分配
* `MicroProfile <https://bitbucket.org/jonasmeyer/microprofile/src/default/>`_  : 跨平台的网络试图分析器
* `UnitTest++ <http://unittest-cpp.sourceforge.net/>`_  : 轻量级的C++单元测试框架

容器
==========================

* `C++ B-Tree <https://code.google.com/p/cpp-btree/>`_  : 基于B树数据结构，实现命令内存容器的模板库
* `Hashmaps <https://github.com/goossaert/hashmap>`_  : C++中开放寻址哈希表算法的实现

游戏引擎
==========================

* `Cocos2d-x <https://cocos2d-x.org/>`_  : 一个跨平台框架，用于构建2D游戏，互动图书，演示和其他图形应用程序。
* `Grit <http://gritengine.com/>`_  : 社区项目，用于构建一个免费的游戏引擎，实现开放的世界3D游戏。
* `lrrlicht <http://irrlicht.sourceforge.net/>`_  : C++语言编写的开源高性能的实时#D引擎
* `PolyCode <http://polycode.org/>`_  : C++实现的用于创建游戏的开源框架（与Lua绑定）。

图形库
==========================

* `bgfx <https://github.com/bkaradzic/bgfx>`_  : 跨平台的渲染库
* `Cairo <http://www.cairographics.org/>`_  : 支持多种输出设备的2D图形库
* `Horde3D <https://github.com/horde3d/Horde3D>`_  : 一个小型的3D渲染和动画引擎
* `magnum <https://github.com/mosra/magnum>`_  : C++11和OpenGL 2D/3D 图形引擎
* `Ogre 3D <https://www.ogre3d.org/>`_  : 用C++编写的一个面向场景，实时，灵活的3D渲染引擎（并非游戏引擎）
* `OpenSceneGraph <http://www.openscenegraph.org/>`_  : 具有高性能的开源3D图形工具包
* `Panda3D <https://www.panda3d.org/>`_  : 用于3D渲染和游戏开发的框架，用Python和C++编写。
* `Skia <https://github.com/google/skia>`_  : 用于绘制文字，图形和图像的完整的2D图形库
* `urho3d <https://github.com/urho3d/Urho3D>`_  : 跨平台的渲染和游戏引擎。

图像处理
==========================

* `Boost.GIL <https://www.boost.org/doc/libs/1_56_0/libs/gil/doc/index.html>`_  : 通用图像库
* `CImg <https://sourceforge.net/projects/cimg/>`_  : 用于图像处理的小型开源C++工具包
* `FreeImage <http://freeimage.sourceforge.net/>`_  : 开源库，支持现在多媒体应用所需的通用图片格式和其他格式。
* `GDCM <http://gdcm.sourceforge.net/wiki/index.php/Main_Page>`_  : Grassroots DICOM 库
* `ITK <https://itk.org/>`_  : 跨平台的开源图像分析系统
* `Magick++ <http://www.imagemagick.org/script/api.php>`_  : ImageMagick程序的C++接口
* `OpenCV <https://opencv.org/>`_  : 开源计算机视觉类库
* `tesseract-ocr <https://code.google.com/p/tesseract-ocr/>`_  : OCR引擎
* `VIGRA <https://github.com/ukoethe/vigra>`_  : 用于图像分析通用C++计算机视觉库
* `VTK <https://vtk.org/>`_  : 用于3D计算机图形学，图像处理和可视化的开源免费软件系统。

国际化
==========================

* `gettext <http://www.gnu.org/software/gettext/>`_  :  GNU gettext
* `IBM ICU <http://site.icu-project.org/>`_  : 提供Unicode 和全球化支持的C、C++ 和Java库
* `libiconv <http://www.gnu.org/software/libiconv/>`_  : 用于不同字符编码之间的编码转换库

Json库
==========================

* `frozen <https://github.com/cesanta/frozen>`_  : C/C++的Jason解析生成器
* `Jansson <https://github.com/akheron/jansson>`_  : 进行编解码和处理Jason数据的C语言库
* `jbson <https://github.com/chrismanning/jbson>`_  : C++14中构建和迭代BSON data,和Json 文档的库
* `JeayeSON <https://github.com/jeaye/jeayeson>`_  : 非常健全的C++ JSON库，只包含头文件
* `JSON++ <https://github.com/hjiang/jsonxx>`_  : C++ JSON 解析器
* `json-parser <https://github.com/udp/json-parser>`_  : 用可移植的ANSI C编写的JSON解析器，占用内存非常少
* `json11 <https://github.com/dropbox/json11>`_  : 一个迷你的C++11 JSON库
* `jute <https://github.com/amir-s/jute>`_  : 非常简单的C++ JSON解析器
* `ibjson <https://github.com/vincenthz/libjson>`_  : C语言中的JSON解析和打印库，很容易和任何模型集成
* `libjson <https://sourceforge.net/projects/libjson/>`_  : 轻量级的JSON库
* `PicoJSON <https://github.com/kazuho/picojson>`_  : C++中JSON解析序列化，只包含头文件
* `Qt-Json <https://github.com/qt-json/qt-json>`_  : 用于JSON数据和 QVariant层次间的相互解析的简单类
* `QJson <https://github.com/flavio/qjson>`_  : 将JSON数据映射到QVariant对象的基于Qt的库
* `RepidJSON <https://github.com/Tencent/rapidjson>`_  : 用于C++的快速JSON 解析生成器，包含SAX和DOM两种风格的API

日志
==========================

* `Boost.Log <http://www.boost.org/doc/libs/1_56_0/libs/log/doc/html/index.html>`_  : 设计非常模块化，并且具有扩展性
* `easyloggingpp <https://github.com/zuhd-org/easyloggingpp>`_  : C++日志库，只包含单一的头文件。
* `Log4cpp <http://log4cpp.sourceforge.net/>`_  : 一系列C++类库，灵活添加日志到文件，系统日志，IDSA和其他地方。
* `templog <http://www.templog.org/>`_  : 轻量级C++库，可以添加日志到你的C++应用程序中

机器学习，人工智能
==========================

* `btsk <https://github.com/aigamedev/btsk>`_  : 游戏行为树启动器工具
* `Evolving Objects <http://eodev.sourceforge.net/>`_  : 基于模板的，ANSI C++演化计算库，能够帮助你非常快速地编写出自己的随机优化算法。
* `Caffe <https://github.com/BVLC/caffe>`_  : 快速的神经网络框架
* `CCV <https://github.com/liuliu/ccv>`_  : 以C语言为核心的现代计算机视觉库
* `mlpack <http://www.mlpack.org/>`_  :  可扩展的C++机器学习库
* `OpenCV <https://github.com/opencv/opencv>`_  : 开源计算机视觉库
* `Recommender <https://github.com/GHamrouni/Recommender>`_  : 使用协同过滤进行产品推荐/建议的C语言库。
* `SHOGUN <https://github.com/shogun-toolbox/shogun>`_  : Shogun 机器学习工具
* `sofia-ml <https://code.google.com/p/sofia-ml/>`_  : 用于机器学习的快速增量算法套件

数学库
==========================

* `Armadillo <http://arma.sourceforge.net/>`_  : 高质量的C++线性代数库，速度和易用性做到了很好的平衡。语法和MatlAB很相似
* `blaze <https://code.google.com/p/blaze-lib/>`_  : 高性能的C++数学库，用于密集和稀疏算法。
* `ceres-solver <http://ceres-solver.org/>`_  : 来自谷歌的C++库，用于建模和解决大型复杂非线性最小平方问题。
* `CGal <http://www.cgal.org/>`_  : 高效，可靠的集合算法集合
* `CML <https://github.com/demianmnave/CML/wiki/The-Configurable-Math-Library>`_  : 用于游戏和图形的免费C++数学库
* `Eigen <http://eigen.tuxfamily.org/index.php?title=Main_Page>`_  : 高级C++模板头文件库，包括线性代数，矩阵，向量操作，数值解决和其他相关的算法。
* `GMTL <http://ggt.sourceforge.net/>`_  : 数学图形模板库是一组广泛实现基本图形的工具。
* `GMP <https://gmplib.org/>`_  : 用于个高精度计算的C/C++库，处理有符号整数，有理数和浮点数。

多媒体库
==========================

* `GStreamer <https://gstreamer.freedesktop.org/>`_  : 构建媒体处理组件图形的库
* `LIVE555 Streaming Media <http://www.live555.com/liveMedia/>`_  : 使用开放标准协议(RTP/RTCP, RTSP, SIP>`_  的多媒体流库
* `libVLC <https://wiki.videolan.org/LibVLC>`_  : libVLC (VLC SDK>`_ 媒体框架
* `QtAV <https://github.com/wang-bin/QtAV>`_  : 基于Qt和FFmpeg的多媒体播放框架，能够帮助你轻而易举地编写出一个播放器
* `SDL <http://www.libsdl.org/>`_  : 简单直控媒体层
* `SFML <http://www.sfml-dev.org/>`_  : 快速，简单的多媒体库

物理学
==========================

* `Box2D <https://code.google.com/p/box2d/>`_  : 2D的游戏物理引擎。
* `Bullet <https://github.com/bulletphysics/bullet3>`_  : 3D的游戏物理引擎。
* `Chipmunk <https://github.com/slembcke/Chipmunk2D>`_  : 快速，轻量级的2D游戏物理库
* `LiquidFun <https://github.com/google/liquidfun>`_  : 2D的游戏物理引擎
* `ODE <http://www.ode.org/>`_  : 开放动力学引擎-开源，高性能库，模拟刚体动力学。
* `ofxBox2D <https://github.com/vanderlin/ofxBox2d>`_  : Box2D开源框架包装器。
* `Simbody <https://github.com/simbody/simbody>`_  : 高性能C++多体动力学/物理库，模拟关节生物力学和机械系统，像车辆，机器人和人体骨骼。

机器人学
==========================

* `MOOS-Ivp <http://moos-ivp.org/>`_  : 一组开源C++模块，提供机器人平台的自主权，尤其是自主的海洋车辆。
* `MRPT <https://www.mrpt.org/>`_  : 移动机器人编程工具包
* `PCL <https://github.com/PointCloudLibrary/pcl>`_  : 点云库是一个独立的，大规模的开放项目，用于2D/3D图像和点云处理。
* `Robotics Library <http://www.roboticslibrary.org/>`_  : 一个独立的C++库，包括机器人动力学，运动规划和控制。
* `RobWork <http://www.robwork.dk/apidoc/nightly/rw/>`_  : 一组C++库的集合，用于机器人系统的仿真和控制。
* `ROS <http://wiki.ros.org/>`_  : 机器人操作系统，提供了一些库和工具帮助软件开发人员创建机器人应用程序。

脚本
==========================

* `ChaiScript <https://github.com/ChaiScript/ChaiScript/>`_  : 用于C++的易于使用的嵌入式脚本语言。
* `Lua <http://www.lua.org/>`_  : 用于配置文件和基本应用程序脚本的小型快速脚本引擎。
* `luacxx <https://github.com/dafrito/luacxx>`_  : 用于创建Lua绑定的C++ 11 API
* `SWIG <http://www.swig.org/>`_  : 一个可以让你的C++代码链接到JavaScript，Perl，PHP，Python，Tcl和Ruby的包装器/接口生成器
* `V7 <https://github.com/cesanta/v7>`_  : 嵌入式的JavaScript 引擎。
* `V8 <http://code.google.com/p/v8/>`_  : 谷歌的快速JavaScript引擎，可以被嵌入到任何C++应用程序中。

序列化
==========================

* `Cap'n Proto <https://capnproto.org/>`_  : 快速数据交换格式和RPC系统。
* `cereal <https://github.com/USCiLab/cereal>`_  : C++11 序列化库
* `FlatBuffers <https://github.com/google/flatbuffers>`_  : 内存高效的序列化库
* `MessagePack <https://github.com/msgpack/msgpack-c>`_  : C/C++的高效二进制序列化库，例如 JSON
* `ProtoBuf <http://code.google.com/p/protobuf/>`_  : 协议缓冲，谷歌的数据交换格式。
* `SimpleBinaryEncoding <https://github.com/real-logic/simple-binary-encoding>`_  : 用于低延迟应用程序的对二进制格式的应用程序信息的编码和解码。
* `Thrift <https://thrift.apache.org/>`_  : 高效的跨语言IPC/RPC，用于C++，Java，Python，PHP，C#和其它多种语言中，最初由Facebook开发。

视频库
==========================

* `libvpx <http://www.webmproject.org/code/>`_  : VP8/VP9编码解码SDK
* `FFMpeg <https://www.ffmpeg.org/>`_  : 一个完整的，跨平台的解决方案，用于记录，转换视频和音频流。
* `libde265 <https://github.com/strukturag/libde265>`_  : 开放的h.265视频编解码器的实现。
* `OpenH264 <https://github.com/cisco/openh264>`_  : 开源H.364 编解码器。
* `Theora <https://www.theora.org/>`_  : 免费开源的视频压缩格式。

XML库
==========================

* `LibXml++ <http://libxmlplusplus.sourceforge.net/>`_  : C++的xml解析器
* `PugiXML <https://pugixml.org/>`_  : 用于C++的，支持XPath的轻量级，简单快速的XML解析器。
* `RapidXML <http://rapidxml.sourceforge.net/>`_  : 试图创建最快速的XML解析器，同时保持易用性，可移植性和合理的W3C兼容性。
* `TinyXML <https://sourceforge.net/projects/tinyxml/>`_  : 简单小型的C++XML解析器，可以很容易地集成到其它项目中。
* `TinyXML2 <https://github.com/leethomason/tinyxml2>`_  : 简单快速的C++CML解析器，可以很容易集成到其它项目中。
* `TinyXML++ <https://code.google.com/p/ticpp/>`_  : TinyXML的一个全新的接口，使用了C++的许多许多优势，模板，异常和更好的异常处理。
* `Xerces-C++ <http://xerces.apache.org/xerces-c/>`_  : 用可移植的C++的子集编写的XML验证解析器。

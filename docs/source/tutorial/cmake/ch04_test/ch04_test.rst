.. highlight:: c++

.. default-domain:: cpp

==========================
第4章 创建和运行测试
==========================

CTest是CMake的测试工具

4.1 创建一个简单的单元测试
--------------------------
**准备工作**

代码示例由三个文件组成。实现源文件sum_integs.cpp对整数向量进行求和，并返回累加结果：

.. code-block:: c++

  #include "sum_integers.hpp"
  #include <vector>

  int sum_integers(const std::vector<int> integers) 
  {
      auto sum = 0;
      for (auto i : integers) {
          sum += i;
      }
      return sum;
  }

这个示例是否是优雅的实现并不重要，接口以sum_integers的形式导出。接口在sum_integers.hpp文件中声明，详情如下:

.. code-block:: c++

  #pragma once
  #include <vector>
  int sum_integers(const std::vector<int> integers);

最后，main函数在main.cpp中定义，从argv[]中收集命令行参数，将它们转换成整数向量，调用sum_integers函数，并将结果打印到输出中:

.. code-block:: c++

  #include "sum_integers.hpp"
  #include <iostream>
  #include <string>
  #include <vector>

  // we assume all arguments are integers and we sum them up
  // for simplicity we do not verify the type of arguments
  int main(int argc, char *argv[]) {
      std::vector<int> integers;
      for (auto i = 1; i < argc; i++) {
          integers.push_back(std::stoi(argv[i]));
      }
      auto sum = sum_integers(integers);
      std::cout << sum << std::endl;
  }

测试这段代码使用C++实现(test.cpp)，Bash shell脚本实现(test.sh)和Python脚本实现(test.py)，只要实现可以返回一个零或非零值，从而CMake可以解释为成功或失败。

C++例子(test.cpp)中，我们通过调用sum_integers来验证1 + 2 + 3 + 4 + 5 = 15：

.. code-block:: c++

  #include "sum_integers.hpp"
  #include <vector>

  int main() 
  {
      auto integers = {1, 2, 3, 4, 5};
    if (sum_integers(integers) == 15) {
          return 0;
      } else {
          return 1;
      }
  }

Bash shell脚本调用可执行文件：

.. code-block:: bash

  #!/usr/bin/env bash
  EXECUTABLE=$1
  OUTPUT=$($EXECUTABLE 1 2 3 4)
  if [ "$OUTPUT" = "10" ]
  then
      exit 0
  else
      exit 1
  fi

此外，Python脚本调用可执行文件(使用--executable命令行参数传递)，并使用--short命令行参数执行：

.. code-block:: python

  import subprocess
  import argparse
  # test script expects the executable as argument
  parser = argparse.ArgumentParser()
  parser.add_argument('--executable',
                                          help='full path to executable')
  parser.add_argument('--short',
                                          default=False,
                      action='store_true',
                      help='run a shorter test')
  args = parser.parse_args()
  def execute_cpp_code(integers):
      result = subprocess.check_output([args.executable] + integers)
      return int(result)
  if args.short:
      # we collect [1, 2, ..., 100] as a list of strings
      result = execute_cpp_code([str(i) for i in range(1, 101)])
      assert result == 5050, 'summing up to 100 failed'
  else:
      # we collect [1, 2, ..., 1000] as a list of strings
      result = execute_cpp_code([str(i) for i in range(1, 1001)])
      assert result == 500500, 'summing up to 1000 failed'

**具体实施**

现在，我们将逐步描述如何为项目设置测试：

1 对于这个例子，我们需要C++11支持，可用的Python解释器，以及Bash shell:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(PythonInterp REQUIRED)
  find_program(BASH_EXECUTABLE NAMES bash REQUIRED)

2 然后，定义库及主要可执行文件的依赖关系，以及测试可执行文件：

.. code-block:: cmake

  # example library
  add_library(sum_integers sum_integers.cpp)
  # main code
  add_executable(sum_up main.cpp)
  target_link_libraries(sum_up sum_integers)
  # testing binary
  add_executable(cpp_test test.cpp)
  target_link_libraries(cpp_test sum_integers)

3 最后，打开测试功能并定义四个测试。最后两个测试， 调用相同的Python脚本，先没有任何命令行参数，再使用--short：

.. code-block:: cmake

  enable_testing()
  add_test(
    NAME bash_test
    COMMAND ${BASH_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.sh $<TARGET_FILE:sum_up>
    )
  add_test(
    NAME cpp_test
    COMMAND $<TARGET_FILE:cpp_test>
    )
  add_test(
    NAME python_test_long
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py --executable $<TARGET_FILE:sum_up>
    )
  add_test(
    NAME python_test_short
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py --short --executable $<TARGET_FILE:sum_up>
    )

4 现在，我们已经准备好配置和构建代码。先手动进行测试：

.. code-block:: bash

  mkdir -p build
  cd build
  cmake ..
  cmake --build .
  ./sum_up 1 2 3 4 5

  15

5 然后，我们可以用ctest运行测试集：

.. code-block:: bash

  ctest
  Test project /home/user/cmake-recipes/chapter-04/recipe-01/cxx-example/build
  Start 1: bash_test
  1/4 Test #1: bash_test ........................ Passed 0.01 sec
  Start 2: cpp_test
  2/4 Test #2: cpp_test ......................... Passed 0.00 sec
  Start 3: python_test_long
  3/4 Test #3: python_test_long ................. Passed 0.06 sec
  Start 4: python_test_short
  4/4 Test #4: python_test_short ................ Passed 0.05 sec
  100% tests passed, 0 tests failed out of 4
  Total Test time (real) = 0.12 sec

6 还应该尝试中断实现，以验证测试集是否能捕捉到更改。

**工作原理**

这里的两个关键命令：

* enable_testing()，测试这个目录和所有子文件夹(因为我们把它放在主CMakeLists.txt)。
*  add_test()，定义了一个新的测试，并设置测试名称和运行命令。

.. code-block:: cmake

  add_test(
    NAME cpp_test
    COMMAND $<TARGET_FILE:cpp_test>
    )


上面的例子中，使用了生成器表达式:$<TARGET_FILE:cpp_test>。生成器表达式，是在生成构建系统生成时的表达式。我们将在第5章第9节中详细地描述生成器表达式。此时，我们可以声明$<TARGET_FILE:cpp_test>变量，将使用cpp_test可执行目标的完整路径进行替换。

生成器表达式在测试时非常方便，因为不必显式地将可执行程序的位置和名称，可以硬编码到测试中。以一种可移植的方式实现这一点非常麻烦，因为可执行文件和可执行后缀(例如，Windows上是.exe后缀)的位置在不同的操作系统、构建类型和生成器之间可能有所不同。使用生成器表达式，我们不必显式地了解位置和名称。

也可以将参数传递给要运行的test命令，例如：

.. code-block:: cmake

  add_test(
    NAME python_test_short
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py --short --executable $<TARGET_FILE:sum_up>
    )

这个例子中，我们按顺序运行测试，并展示如何缩短总测试时间并行执行测试(第8节)，执行测试用例的子集(第9节)。这里，可以自定义测试命令，可以以任何编程语言运行测试集。CTest关心的是，通过命令的返回码测试用例是否通过。CTest遵循的标准约定是，返回零意味着成功，非零返回意味着失败。可以返回零或非零的脚本，都可以做测试用例。

既然知道了如何定义和执行测试，那么了解如何诊断测试失败也很重要。为此，我们可以在代码中引入一个bug，让所有测试都失败:

.. code-block:: bash

  Start 1: bash_test
  1/4 Test #1: bash_test ........................***Failed 0.01 sec
      Start 2: cpp_test
  2/4 Test #2: cpp_test .........................***Failed 0.00 sec
      Start 3: python_test_long
  3/4 Test #3: python_test_long .................***Failed 0.06 sec
      Start 4: python_test_short
  4/4 Test #4: python_test_short ................***Failed 0.06 sec
  0% tests passed, 4 tests failed out of 4
  Total Test time (real) = 0.13 sec
  The following tests FAILED:
  1 - bash_test (Failed)
  2 - cpp_test (Failed)
  3 - python_test_long (Failed)
  4 - python_test_short (Failed)
  Errors while running CTest


如果我们想了解更多，可以查看文件test/Temporary/lasttestsfailure.log。这个文件包含测试命令的完整输出，并且在分析阶段，要查看的第一个地方。使用以下CLI开关，可以从CTest获得更详细的测试输出：

* --output-on-failure:将测试程序生成的任何内容打印到屏幕上，以免测试失败。
* -v:将启用测试的详细输出。
* -vv:启用更详细的输出。

CTest提供了一个非常方快捷的方式，可以重新运行以前失败的测试；要使用的CLI开关是--rerun-failed，在调试期间非常有用。

**更多信息**

考虑以下定义:

.. code-block:: cmake

    add_test(
      NAME python_test_long
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py --executable $<TARGET_FILE:sum_up>
      )

前面的定义可以通过显式指定脚本运行的WORKING_DIRECTORY重新表达，如下:

.. code-block:: cmake

  add_test(
    NAME python_test_long
    COMMAND ${PYTHON_EXECUTABLE} test.py --executable $<TARGET_FILE:sum_up>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

测试名称可以包含/字符，按名称组织相关测试也很有用，例如：

.. code-block:: cmake

  add_test(
    NAME python/long
    COMMAND ${PYTHON_EXECUTABLE} test.py --executable $<TARGET_FILE:sum_up>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

有时候，我们需要为测试脚本设置环境变量。这可以通过set_tests_properties实现:

.. code-block:: cmake

  set_tests_properties(python_test
    PROPERTIES
      ENVIRONMENT
        ACCOUNT_MODULE_PATH=${CMAKE_CURRENT_SOURCE_DIR}
        ACCOUNT_HEADER_FILE=${CMAKE_CURRENT_SOURCE_DIR}/account/account.h
        ACCOUNT_LIBRARY_FILE=$<TARGET_FILE:account>
    )

这种方法在不同的平台上并不总可行，CMake提供了解决这个问题的方法。下面的代码片段与上面给出的代码片段相同，在执行实际的Python测试脚本之前，通过CMAKE_COMMAND调用CMake来预先设置环境变量:

.. code-block:: cmake

  add_test(
    NAME
        python_test
    COMMAND
      ${CMAKE_COMMAND} -E env
      ACCOUNT_MODULE_PATH=${CMAKE_CURRENT_SOURCE_DIR}
      ACCOUNT_HEADER_FILE=${CMAKE_CURRENT_SOURCE_DIR}/account/account.h
      ACCOUNT_LIBRARY_FILE=$<TARGET_FILE:account>
      ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/account/test.py
    )


同样，要注意使用生成器表达式$<TARGET_FILE:account>来传递库文件的位置。

我们已经使用ctest命令执行测试，CMake还将为生成器创建目标(Unix Makefile生成器为make test，Ninja工具为ninja test，或者Visual Studio为RUN_TESTS)。这意味着，还有另一种(几乎)可移植的方法来运行测试：

.. code-block:: bash

  cmake --build . --target test

不幸的是，当使用Visual Studio生成器时，我们需要使用RUN_TESTS来代替:

.. code-block:: bash

  cmake --build . --target RUN_TESTS

.. NOTE:: 

  ctest提供了丰富的命令行参数。其中一些内容将在以后的示例中探讨。要获得完整的列表，需要使用ctest --help来查看。命令cmake --help-manual ctest会将向屏幕输出完整的ctest手册。


4.2 使用Catch2库进行单元测试
-------------------------------

.. NOTE:: 

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-02 中找到，包含一个C++的示例。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。
  前面的配置中，使用返回码来表示test.cpp测试的成功或失败。对于简单功能没问题，但是通常情况下，我们想要使用一个测试框架，它提供了相关基础设施来运行更复杂的测试，包括固定方式进行测试，与数值公差的比较，以及在测试失败时输出更好的错误报告。这里，我们用目前比较流行的测试库Catch2( https://github.com/catchorg/Catch2 )来进行演示。这个测试框架有个很好的特性，它可以通过单个头库包含在项目中进行测试，这使得编译和更新框架特别容易。这个配置中，我们将CMake和Catch2结合使用，来测试上一个求和代码。
  我们需要catch.hpp头文件，可以从 https://github.com/catchorg/Catch2 (我们使用的是版本2.0.1)下载，并将它与test.cpp一起放在项目的根目录下。

**准备工作**

main.cpp、sum_integers.cpp和sum_integers.hpp与之前的示例相同，但将更新test.cpp:

.. code-block:: c++

  #include "sum_integers.hpp"
  // this tells catch to provide a main()
  // only do this in one cpp file
  #define CATCH_CONFIG_MAIN
  #include "catch.hpp"
  #include <vector>
  TEST_CASE("Sum of integers for a short vector", "[short]")
  {
    auto integers = {1, 2, 3, 4, 5};
    REQUIRE(sum_integers(integers) == 15);
  }
  TEST_CASE("Sum of integers for a longer vector", "[long]")
  {
    std::vector<int> integers;
    for (int i = 1; i < 1001; ++i)
    {
      integers.push_back(i);
    }
    REQUIRE(sum_integers(integers) == 500500);
  }

catch.hpp头文件可以从https://github.com/catchorg/Catch2 (版本为2.0.1)下载，并将它与test.cpp放在项目的根目录中。

**具体实施**

使用Catch2库，需要修改之前的所使用CMakeList.txt：

保持CMakeLists.txt大多数部分内容不变:

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name and language
  project(recipe-02 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # example library
  add_library(sum_integers sum_integers.cpp)
  # main code
  add_executable(sum_up main.cpp)
  target_link_libraries(sum_up sum_integers)
  # testing binary
  add_executable(cpp_test test.cpp)
  target_link_libraries(cpp_test sum_integers)

对于上一个示例的配置，需要保留一个测试，并重命名它。注意，--success选项可传递给单元测试的可执行文件。这是一个Catch2选项，测试成功时，也会有输出:

.. code-block:: cmake

  enable_testing()
  add_test(
    NAME catch_test
    COMMAND $<TARGET_FILE:cpp_test> --success
    )
  
就是这样！让我们来配置、构建和测试。CTest中，使用-V选项运行测试，以获得单元测试可执行文件的输出:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest -V
  UpdateCTestConfiguration from :/home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/build/DartConfiguration.tcl
  UpdateCTestConfiguration from :/home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/build/DartConfiguration.tcl
  Test project /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/build
  Constructing a list of tests
  Done constructing a list of tests
  Updating test list for fixtures
  Added 0 tests to meet fixture requirements
  Checking test dependency graph...
  Checking test dependency graph end
  test 1
  Start 1: catch_test
  1: Test command: /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/build/cpp_test "--success"
  1: Test timeout computed to be: 10000000
  1:
  1: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  1: cpp_test is a Catch v2.0.1 host application.
  1: Run with -? for options
  1:
  1: ----------------------------------------------------------------
  1: Sum of integers for a short vector
  1: ----------------------------------------------------------------
  1: /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:10
  1: ...................................................................
  1:
  1: /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:12:
  1: PASSED:
  1: REQUIRE( sum_integers(integers) == 15 )
  1: with expansion:
  1: 15 == 15
  1:
  1: ----------------------------------------------------------------
  1: Sum of integers for a longer vector
  1: ----------------------------------------------------------------
  1: /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:15
  1: ...................................................................
  1:
  1: /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:20:
  1: PASSED:
  1: REQUIRE( sum_integers(integers) == 500500 )
  1: with expansion:
  1: 500500 (0x7a314) == 500500 (0x7a314)
  1:
  1: ===================================================================
  1: All tests passed (2 assertions in 2 test cases)
  1:
  1/1 Test #1: catch_test ....................... Passed 0.00 s
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.00 se

我们也可以测试cpp_test的二进制文件，可以直接从Catch2中看到输出:

.. code-block:: bash

  $ ./cpp_test --success`

  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  cpp_test is a Catch v2.0.1 host application.
  Run with -? for options
  -------------------------------------------------------------------
  Sum of integers for a short vector
  -------------------------------------------------------------------
  /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:10
  ...................................................................
  /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:12:
  PASSED:
  REQUIRE( sum_integers(integers) == 15 )
  with expansion:
  15 == 15
  -------------------------------------------------------------------
  Sum of integers for a longer vector
  -------------------------------------------------------------------
  /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:15
  ...................................................................
  /home/user/cmake-cookbook/chapter-04/recipe-02/cxx-example/test.cpp:20:
  PASSED:
  REQUIRE( sum_integers(integers) == 500500 )
  with expansion:
  500500 (0x7a314) == 500500 (0x7a314)
  ===================================================================
  All tests passed (2 assertions in 2 test cases)

Catch2将生成一个可执行文件，还可以尝试执行以下命令，以探索单元测试框架提供的选项:

.. code-block:: bash

  $ ./cpp_test --help

**工作原理**

Catch2是一个单头文件测试框架，所以不需要定义和构建额外的目标。只需要确保CMake能找到catch.hpp，从而构建test.cpp即可。为了方便起见，将它放在与test.cpp相同的目录中，我们可以选择一个不同的位置，并使用target_include_directory指示该位置。另一种方法是将头部封装到接口库中，这可以在Catch2文档中说明( https://github.com/catchorg/catch2/blob/maste/docs/build.systems.md#cmake ):

.. code-block:: cmake

  # Prepare "Catch" library for other executables 
  set(CATCH_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/catch) 
  add_library(Catch
  INTERFACE) 
  target_include_directories(Catch INTERFACE
  ${CATCH_INCLUDE_DIR})

然后，我们对库进行如下链接:

.. code-block:: cmake

  target_link_libraries(cpp_test Catch)

回想一下第3中的讨论，在第1章从简单的可执行库到接口库，是CMake提供的伪目标库，这些伪目标库对于指定项目外部目标的需求非常有用。

**更多信息**

这是一个简单的例子，主要关注CMake。当然，Catch2提供了更多功能。有关Catch2框架的完整文档，可访问 https://github.com/catchorg/Catch2 。

Catch2代码库包含有CMake函数，用于解析Catch测试并自动创建CMake测试，不需要显式地输入add_test()函数，可见 https://github.com/catchorg/Catch2/blob/master/contrib/ParseAndAddCatchTests.cmake 。

4.3 使用Google Test库进行单元测试
----------------------------------

.. NOTE:: 

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-03 中找到，包含一个C++的示例。该示例在CMake 3.11版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。在代码库中，有一个支持CMake 3.5的例子。
  本示例中，我们将演示如何在CMake的帮助下使用Google Test框架实现单元测试。与前一个配置相比，Google Test框架不仅仅是一个头文件，也是一个库，包含两个需要构建和链接的文件。可以将它们与我们的代码项目放在一起，但是为了使代码项目更加轻量级，我们将选择在配置时，下载一个定义良好的Google Test，然后构建框架并链接它。我们将使用较新的FetchContent模块(从CMake版本3.11开始可用)。第8章中会继续讨论FetchContent，在这里将讨论模块在底层是如何工作的，并且还将演示如何使用ExternalProject_Add进行模拟。此示例的灵感来自(改编自) https://cmake.org/cmake/help/v3.11/module/FetchContent.html 示例。

**准备工作**

main.cpp、sum_integers.cpp和sum_integers.hpp与之前相同，修改test.cpp:

.. code-block:: c++

  #include "sum_integers.hpp"
  #include "gtest/gtest.h"
  #include <vector>

  int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }

  TEST(example, sum_zero) {
    auto integers = {1, -1, 2, -2, 3, -3};
    auto result = sum_integers(integers);
    ASSERT_EQ(result, 0);
  }

  TEST(example, sum_five) {
    auto integers = {1, 2, 3, 4, 5};
    auto result = sum_integers(integers);
    ASSERT_EQ(result, 15);
  }

如上面的代码所示，我们显式地将gtest.h，而不将其他Google Test源放在代码项目存储库中，会在配置时使用FetchContent模块下载它们。

**具体实施**
下面的步骤描述了如何设置CMakeLists.txt，使用GTest编译可执行文件及其相应的测试:

与前两个示例相比，CMakeLists.txt的开头基本没有变化，CMake 3.11才能使用FetchContent模块:

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
  # project name and language
  project(recipe-03 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # example library
  add_library(sum_integers sum_integers.cpp)
  # main code
  add_executable(sum_up main.cpp)
  target_link_libraries(sum_up sum_integers)

然后引入一个if，检查ENABLE_UNIT_TESTS。默认情况下，它为ON，但有时需要设置为OFF，以免在没有网络连接时，也能使用Google Test:

.. code-block:: cmake

  option(ENABLE_UNIT_TESTS "Enable unit tests" ON)
  message(STATUS "Enable testing: ${ENABLE_UNIT_TESTS}")
  if(ENABLE_UNIT_TESTS)
      # all the remaining CMake code will be placed here
  endif()

if内部包含FetchContent模块，声明要获取的新内容，并查询其属性:

.. code-block:: cmake

  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.8.0
  )
  FetchContent_GetProperties(googletest)

如果内容还没有获取到，将尝试获取并配置它。这需要添加几个可以链接的目标。本例中，我们对gtest_main感兴趣。该示例还包含一些变通方法，用于使用在Visual Studio下的编译:

.. code-block:: cmake

  if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
    # Prevent GoogleTest from overriding our compiler/linker options
    # when building with Visual Studio
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    # Prevent GoogleTest from using PThreads
    set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)
    # adds the targers: gtest, gtest_main, gmock, gmock_main
    add_subdirectory(
      ${googletest_SOURCE_DIR}
      ${googletest_BINARY_DIR}
      )
    # Silence std::tr1 warning on MSVC
    if(MSVC)
      foreach(_tgt gtest gtest_main gmock gmock_main)
        target_compile_definitions(${_tgt}
          PRIVATE
              "_SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING"
        )
      endforeach()
    endif()
  endif()

然后，使用target_sources和target_link_libraries命令，定义cpp_test可执行目标并指定它的源文件:

.. code-block:: cmake

  add_executable(cpp_test "")
  target_sources(cpp_test
    PRIVATE
        test.cpp
    )
  target_link_libraries(cpp_test
    PRIVATE
      sum_integers
      gtest_main
    )

最后，使用enable_test和add_test命令来定义单元测试:

.. code-block:: cmake

  enable_testing()
  add_test(
    NAME google_test
    COMMAND $<TARGET_FILE:cpp_test>
    )

现在，准备配置、构建和测试项目:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest
  Test project /home/user/cmake-cookbook/chapter-04/recipe-03/cxx-example/build
      Start 1: google_test
  1/1 Test #1: google_test ...................... Passed 0.00 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.00 sec

可以直接运行cpp_test:

.. code-block:: bash

  $ ./cpp_test
  [==========] Running 2 tests from 1 test case.
  [----------] Global test environment set-up.
  [----------] 2 tests from example
  [ RUN ] example.sum_zero
  [ OK ] example.sum_zero (0 ms)
  [ RUN ] example.sum_five
  [ OK ] example.sum_five (0 ms)
  [----------] 2 tests from example (0 ms total)
  [----------] Global test environment tear-down
  [==========] 2 tests from 1 test case ran. (0 ms total)
  [ PASSED ] 2 tests.

**工作原理**

FetchContent模块支持通过ExternalProject模块，在配置时填充内容，并在其3.11版本中成为CMake的标准部分。而ExternalProject_Add()在构建时(见第8章)进行下载操作，这样FetchContent模块使得构建可以立即进行，这样获取的主要项目和外部项目(在本例中为Google Test)仅在第一次执行CMake时调用，使用add_subdirectory可以嵌套。

为了获取Google Test，首先声明外部内容:

.. code-block:: cmake

  include(FetchContent)
    FetchContent_Declare(
        googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG release-1.8.0
  )

本例中，我们获取了一个带有特定标记的Git库(release-1.8.0)，但是我们也可以从Subversion、Mercurial或HTTP(S)源获取一个外部项目。有关可用选项，可参考相应的ExternalProject_Add命令的选项，网址是https://cmake.org/cmake/help/v3.11/module/ExternalProject.html 。

调用FetchContent_Populate()之前，检查是否已经使用FetchContent_GetProperties()命令处理了内容填充；否则，调用FetchContent_Populate()超过一次后，就会抛出错误。

FetchContent_Populate(googletest)用于填充源并定义googletest_SOURCE_DIR和googletest_BINARY_DIR，可以使用它们来处理Google Test项目(使用add_subdirectory()，因为它恰好也是一个CMake项目):

.. code-block:: cmake

  add_subdirectory(
    ${googletest_SOURCE_DIR}
    ${googletest_BINARY_DIR}
    )

前面定义了以下目标：gtest、gtest_main、gmock和gmock_main。这个配置中，作为单元测试示例的库依赖项，我们只对gtest_main目标感兴趣：

.. code-block:: cmake

  target_link_libraries(cpp_test
    PRIVATE
      sum_integers
      gtest_main
  )

构建代码时，可以看到如何正确地对Google Test进行配置和构建。有时，我们希望升级到更新的Google Test版本，这时需要更改的唯一一行就是详细说明GIT_TAG的那一行。

**更多信息**

了解了FetchContent及其构建时的近亲ExternalProject_Add，我们将在第8章中重新讨论这些命令。有关可用选项的详细讨论，可参考https://cmake.org/cmake/help/v3.11/module/FetchContent.html 。

本示例中，我们在配置时获取源代码，也可以将它们安装在系统环境中，并使用FindGTest模块来检测库和头文件(https://cmake.org/cmake/help/v3.5/module/FindTest.html )。从3.9版开始，CMake还提供了一个Google Test模块(https://cmake.org/cmake/help/v3.9/module/GoogleTest.html )，它提供了一个gtest_add_tests函数。通过搜索Google Test宏的源代码，可以使用此函数自动添加测试。

当然，Google Test有许多有趣的的特性，可在 https://github.com/google/googletest 查看。


4.4 使用Boost Test进行单元测试
---------------------------------

.. NOTE:: 
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-04 中找到，包含一个C++的示例。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。
  Boost Test是在C++社区中，一个非常流行的单元测试框架。本例中，我们将演示如何使用Boost Test，对求和示例代码进行单元测试。

**准备工作**

main.cpp、sum_integers.cpp和sum_integers.hpp与之前的示例相同，将更新test.cpp作为使用Boost Test库进行的单元测试：

.. code-block:: c++

  #include "sum_integers.hpp"
  #include <vector>
  #define BOOST_TEST_MODULE example_test_suite
  #include <boost/test/unit_test.hpp>
  BOOST_AUTO_TEST_CASE(add_example)
  {
    auto integers = {1, 2, 3, 4, 5};
    auto result = sum_integers(integers);
    BOOST_REQUIRE(result == 15);
  }

**具体实施**

以下是使用Boost Test构建项目的步骤:

先从CMakeLists.txt开始:

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name and language
  project(recipe-04 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # example library
  add_library(sum_integers sum_integers.cpp)
  # main code
  add_executable(sum_up main.cpp)
  target_link_libraries(sum_up sum_integers)

检测Boost库并将cpp_test链接到它:

.. code-block:: cmake

  find_package(Boost 1.54 REQUIRED COMPONENTS unit_test_framework)
  add_executable(cpp_test test.cpp)
  target_link_libraries(cpp_test
    PRIVATE
      sum_integers
      Boost::unit_test_framework
    )
  # avoid undefined reference to "main" in test.cpp
  target_compile_definitions(cpp_test
    PRIVATE
        BOOST_TEST_DYN_LINK
    )

最后，定义单元测试:

.. code-block:: cmake

  enable_testing()
  add_test(
    NAME boost_test
    COMMAND $<TARGET_FILE:cpp_test>
    )

下面是需要配置、构建和测试代码的所有内容:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest
  Test project /home/user/cmake-recipes/chapter-04/recipe-04/cxx-example/build
  Start 1: boost_test
  1/1 Test #1: boost_test ....................... Passed 0.01 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.01 sec
  $ ./cpp_test
  Running 1 test case...
  *** No errors detected

**工作原理**

使用find_package来检测Boost的unit_test_framework组件(参见第3章，第8节)。我们认为这个组件是REQUIRED的，如果在系统环境中找不到它，配置将停止。cpp_test目标需要知道在哪里可以找到Boost头文件，并且需要链接到相应的库；它们都由IMPORTED库目标Boost::unit_test_framework提供，该目标由find_package设置。

更多信息
本示例中，我们假设系统上安装了Boost。或者，我们可以在编译时获取并构建Boost依赖项。然而，Boost不是轻量级依赖项。我们的示例代码中，我们只使用了最基本的设施，但是Boost提供了丰富的特性和选项，有感兴趣的读者可以去这里看看：http://www.boost.org/doc/libs/1_65_1/libs/test/doc/html/index.html 。


4.5 使用动态分析来检测内存缺陷
------------------------------

.. NOTE:: 

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-05 中找到，包含一个C++的示例。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

内存缺陷：写入或读取越界，或者内存泄漏(已分配但从未释放的内存)，会产生难以跟踪的bug，最好尽早将它们检查出来。Valgrind( http://valgrind.org )是一个通用的工具，用来检测内存缺陷和内存泄漏。本节中，我们将在使用CMake/CTest测试时使用Valgrind对内存问题进行警告。

**准备工作**

对于这个配置，需要三个文件。第一个是测试的实现(我们可以调用文件leaky_implementation.cpp):

.. code-block:: c++

  #include "leaky_implementation.hpp"
  int do_some_work() {
    // we allocate an array
    double *my_array = new double[1000];
    // do some work
    // ...
    // we forget to deallocate it
    // delete[] my_array;
    return 0;
  }

还需要相应的头文件(leaky_implementation.hpp):

#pragma once
int do_some_work();
并且，需要测试文件(test.cpp):

.. code-block:: c++

  #include "leaky_implementation.hpp"
  int main() {
    int return_code = do_some_work();
    return return_code;
  }

我们希望测试通过，因为return_code硬编码为0。这里我们也期望检测到内存泄漏，因为my_array没有释放。

**具体实施**

下面展示了如何设置CMakeLists.txt来执行代码动态分析:

我们首先定义CMake最低版本、项目名称、语言、目标和依赖关系:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-05 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_library(example_library leaky_implementation.cpp)
  add_executable(cpp_test test.cpp)
  target_link_libraries(cpp_test example_library)

然后，定义测试目标，还定义了MEMORYCHECK_COMMAND:

.. code-block:: cmake

  find_program(MEMORYCHECK_COMMAND NAMES valgrind)
  set(MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full")
  # add memcheck test action
  include(CTest)
  enable_testing()
  add_test(
    NAME cpp_test
    COMMAND $<TARGET_FILE:cpp_test>
    )

运行测试集，报告测试通过情况，如下所示:

.. code-block:: bash

  $ ctest
  Test project /home/user/cmake-recipes/chapter-04/recipe-05/cxx-example/build
  Start 1: cpp_test
  1/1 Test #1: cpp_test ......................... Passed 0.00 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.00 sec

现在，我们希望检查内存缺陷，可以观察到被检测到的内存泄漏:

.. code-block:: bash

  $ ctest -T memcheck
  Site: myhost
  Build name: Linux-c++
  Create new tag: 20171127-1717 - Experimental
  Memory check project /home/user/cmake-recipes/chapter-04/recipe-05/cxx-example/build
  Start 1: cpp_test
  1/1 MemCheck #1: cpp_test ......................... Passed 0.40 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.40 sec
  -- Processing memory checking output:
  1/1 MemCheck: #1: cpp_test ......................... Defects: 1
  MemCheck log files can be found here: ( * corresponds to test number)
  /home/user/cmake-recipes/chapter-04/recipe-05/cxx-example/build/Testing/Temporary/MemoryChecker.*.log
  Memory checking results:
  Memory Leak - 1

最后一步，应该尝试修复内存泄漏，并验证ctest -T memcheck没有报告错误。

**工作原理**

使用find_program(MEMORYCHECK_COMMAND NAMES valgrind)查找valgrind，并将MEMORYCHECK_COMMAND设置为其绝对路径。我们显式地包含CTest模块来启用memcheck测试操作，可以使用CTest -T memcheck来启用这个操作。此外，使用set(MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full")，将相关参数传递给Valgrind。内存检查会创建一个日志文件，该文件可用于详细记录内存缺陷信息。

.. NOTE::

  一些工具，如代码覆盖率和静态分析工具，可以进行类似地设置。然而，其中一些工具的使用更加复杂，因为需要专门的构建和工具链。Sanitizers就是这样一个例子。有关更多信息，请参见https://github.com/arsenm/sanitizers-cmake 。另外，请参阅第14章，其中讨论了AddressSanitizer和ThreadSanitizer。


4.6 预期测试失败
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-06 中找到，包含一个C++的示例。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

理想情况下，我们希望所有的测试能在每个平台上通过。然而，也可能想要测试预期的失败或异常是否会在受控的设置中进行。这种情况下，我们将把预期的失败定义为成功。我们认为，这通常应该交给测试框架(例如：Catch2或Google Test)的任务，它应该检查预期的失败并向CMake报告成功。但是，在某些情况下，您可能希望将测试的非零返回代码定义为成功；换句话说，您可能想要颠倒成功和失败的定义。在本示例中，我们将演示这种情况。

**准备工作**

这个配置的测试用例是一个很小的Python脚本(test.py)，它总是返回1，CMake将其解释为失败:

.. code-block:: python

  import sys
  # simulate a failing test
  sys.exit(1)

**实施步骤**

如何编写CMakeLists.txt来完成我们的任务:

这个示例中，不需要任何语言支持从CMake，但需要Python:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-06 LANGUAGES NONE)
  find_package(PythonInterp REQUIRED)

然后，定义测试并告诉CMake，测试预期会失败:

.. code-block:: cmake

  enable_testing()
  add_test(example ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py)
  set_tests_properties(example PROPERTIES WILL_FAIL true)

最后，报告是一个成功的测试，如下所示:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest
  Test project /home/user/cmake-recipes/chapter-04/recipe-06/example/build
  Start 1: example
  1/1 Test #1: example .......................... Passed 0.00 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.01 sec

**工作原理**

使用set_tests_properties(example PROPERTIES WILL_FAIL true)，将属性WILL_FAIL设置为true，这将转换成功与失败。但是，这个特性不应该用来临时修复损坏的测试。

4.7 使用超时测试运行时间过长的测试
-----------------------------------

.. NOTE::
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-07 中找到。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

理想情况下，测试集应该花很短的时间进行，以便开发人员经常运行测试，并使每个提交(变更集)进行测试成为可能(或更容易)。然而，有些测试可能会花费更长的时间或者被卡住(例如，由于高文件I/O负载)，我们可能需要设置超时来终止耗时过长的测试，它们延迟了整个测试，并阻塞了部署管道。本示例中，我们将演示一种设置超时的方法，可以针对每个测试设置不同的超时。

**准备工作**

这个示例是一个Python脚本(test.py)，它总是返回0。为了保持这种简单性，并保持对CMake方面的关注，测试脚本除了等待两秒钟外什么也不做。实际中，这个测试脚本将执行更有意义的工作:

.. code-block:: python

  import sys
  import time
  # wait for 2 seconds
  time.sleep(2)
  # report success
  sys.exit(0)

**具体实施**

我们需要通知CTest终止测试，如下:

1 我们定义项目名称，启用测试，并定义测试:

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name
  project(recipe-07 LANGUAGES NONE)
  # detect python
  find_package(PythonInterp REQUIRED)
  # define tests
  enable_testing()
  # we expect this test to run for 2 seconds
  add_test(example ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py)

2 另外，我们为测试指定时限，设置为10秒:

.. code-block:: cmake

  set_tests_properties(example PROPERTIES TIMEOUT 10)

3 知道了如何进行配置和构建，并希望测试能够通过:

.. code-block:: bash

  $ ctest
  Test project /home/user/cmake-recipes/chapter-04/recipe-07/example/build
  Start 1: example
  1/1 Test #1: example .......................... Passed 2.01 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 2.01 sec

4 现在，为了验证超时是否有效，我们将test.py中的sleep命令增加到11秒，并重新运行测试:

.. code-block:: bash

  $ ctest
  Test project /home/user/cmake-recipes/chapter-04/recipe-07/example/build
  Start 1: example
  1/1 Test #1: example ..........................***Timeout 10.01 sec
  0% tests passed, 1 tests failed out of 1
  Total Test time (real) = 10.01 sec
  The following tests FAILED:
  1 - example (Timeout)
  Errors while running CTest

**工作原理**

TIMEOUT是一个方便的属性，可以使用set_tests_properties为单个测试指定超时时间。如果测试运行超过了这个设置时间，不管出于什么原因(测试已经停止或者机器太慢)，测试将被终止并标记为失败。

4.8 并行测试
--------------------------

.. NOTE::
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-08 中找到。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

大多数现代计算机都有4个或更多个CPU核芯。CTest有个非常棒的特性，能够并行运行测试，如果您有多个可用的核。这可以减少测试的总时间，而减少总测试时间才是真正重要的，从而开发人员频繁地进行测试。本示例中，我们将演示这个特性，并讨论如何优化测试以获得最大的性能。

其他测试可以进行相应地表示，我们把这些测试脚本放在CMakeLists.txt同目录下面的test目录中。

**准备工作**

我们假设测试集包含标记为a, b，…，j的测试用例，每一个都有特定的持续时间:

测试用例	该单元的耗时
a, b, c, d	0.5
e, f, g	1.5
h	2.5
i	3.5
j	4.5


时间单位可以是分钟，但是为了保持简单和简短，我们将使用秒。为简单起见，我们可以用Python脚本表示test a，它消耗0.5个时间单位:

.. code-block:: python

  import sys
  import time
  # wait for 0.5 seconds
  time.sleep(0.5)
  # finally report success
  sys.exit(0)

其他测试同理。我们将把这些脚本放在CMakeLists.txt下面，一个名为test的目录中。

**具体实施**

对于这个示例，我们需要声明一个测试列表，如下:

CMakeLists.txt非常简单：

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name
  project(recipe-08 LANGUAGES NONE)
  # detect python
  find_package(PythonInterp REQUIRED)
  # define tests
  enable_testing()
  add_test(a ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/a.py)
  add_test(b ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/b.py)
  add_test(c ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/c.py)
  add_test(d ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/d.py)
  add_test(e ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/e.py)
  add_test(f ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/f.py)
  add_test(g ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/g.py)
  add_test(h ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/h.py)
  add_test(i ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/i.py)
  add_test(j ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/j.py)

我们可以配置项目，使用ctest运行测试，总共需要17秒:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ ctest
  Start 1: a
  1/10 Test #1: a ................................ Passed 0.51 sec
  Start 2: b
  2/10 Test #2: b ................................ Passed 0.51 sec
  Start 3: c
  3/10 Test #3: c ................................ Passed 0.51 sec
  Start 4: d
  4/10 Test #4: d ................................ Passed 0.51 sec
  Start 5: e
  5/10 Test #5: e ................................ Passed 1.51 sec
  Start 6: f
  6/10 Test #6: f ................................ Passed 1.51 sec
  Start 7: g
  7/10 Test #7: g ................................ Passed 1.51 sec
  Start 8: h
  8/10 Test #8: h ................................ Passed 2.51 sec
  Start 9: i
  9/10 Test #9: i ................................ Passed 3.51 sec
  Start 10: j
  10/10 Test #10: j ................................ Passed 4.51 sec
  100% tests passed, 0 tests failed out of 10
  Total Test time (real) = 17.11 sec

现在，如果机器有4个内核可用，我们可以在不到5秒的时间内在4个内核上运行测试集:

.. code-block:: bash

  $ ctest --parallel 4
  Start 10: j
  Start 9: i
  Start 8: h
  Start 5: e
  1/10 Test #5: e ................................ Passed 1.51 sec
  Start 7: g
  2/10 Test #8: h ................................ Passed 2.51 sec
  Start 6: f
  3/10 Test #7: g ................................ Passed 1.51 sec
  Start 3: c
  4/10 Test #9: i ................................ Passed 3.63 sec
  5/10 Test #3: c ................................ Passed 0.60 sec
  Start 2: b
  Start 4: d
  6/10 Test #6: f ................................ Passed 1.51 sec
  7/10 Test #4: d ................................ Passed 0.59 sec
  8/10 Test #2: b ................................ Passed 0.59 sec
  Start 1: a
  9/10 Test #10: j ................................ Passed 4.51 sec
  10/10 Test #1: a ................................ Passed 0.51 sec
  100% tests passed, 0 tests failed out of 10
  Total Test time (real) = 4.74 sec

**工作原理**
可以观察到，在并行情况下，测试j、i、h和e同时开始。当并行运行时，总测试时间会有显著的减少。观察ctest --parallel 4的输出，我们可以看到并行测试运行从最长的测试开始，最后运行最短的测试。从最长的测试开始是一个非常好的策略。这就像打包移动的盒子：从较大的项目开始，然后用较小的项目填补空白。a-j测试在4个核上的叠加比较，从最长的开始，如下图所示:

.. code-block:: bash

  --> time
  core 1: jjjjjjjjj
  core 2: iiiiiiibd
  core 3: hhhhhggg
  core 4: eeefffac

按照定义测试的顺序运行，运行结果如下:

.. code-block:: bash

  --> time
  core 1: aeeeiiiiiii
  core 2: bfffjjjjjjjjj
  core 3: cggg
  core 4: dhhhhh

按照定义测试的顺序运行测试，总的来说需要更多的时间，因为这会让2个核大部分时间处于空闲状态(这里的核3和核4)。CMake知道每个测试的时间成本，是因为我们先顺序运行了测试，将每个测试的成本数据记录在test/Temporary/CTestCostData.txt文件中:

.. code-block:: bash

  a 1 0.506776
  b 1 0.507882
  c 1 0.508175
  d 1 0.504618
  e 1 1.51006
  f 1 1.50975
  g 1 1.50648
  h 1 2.51032
  i 1 3.50475
  j 1 4.51111

如果在配置项目之后立即开始并行测试，它将按照定义测试的顺序运行测试，在4个核上的总测试时间明显会更长。这意味着什么呢？这意味着，我们应该减少的时间成本来安排测试？这是一种决策，但事实证明还有另一种方法，我们可以自己表示每次测试的时间成本:

.. code-block:: cmake

  add_test(a ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/a.py)
  add_test(b ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/b.py)
  add_test(c ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/c.py)
  add_test(d ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/d.py)
  set_tests_properties(a b c d PROPERTIES COST 0.5)
  add_test(e ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/e.py)
  add_test(f ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/f.py)
  add_test(g ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/g.py)
  set_tests_properties(e f g PROPERTIES COST 1.5)
  add_test(h ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/h.py)
  set_tests_properties(h PROPERTIES COST 2.5)
  add_test(i ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/i.py)
  set_tests_properties(i PROPERTIES COST 3.5)
  add_test(j ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/j.py)
  set_tests_properties(j PROPERTIES COST 4.5)

成本参数可以是一个估计值，也可以从test/Temporary/CTestCostData.txt中提取。

**更多信息**

除了使用ctest --parallel N，还可以使用环境变量CTEST_PARALLEL_LEVEL将其设置为所需的级别。

4.9 运行测试子集
--------------------------

.. NOTE::
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-09 中找到。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

前面的示例中，我们学习了如何在CMake的帮助下并行运行测试，并讨论了从最长的测试开始是最高效的。虽然，这种策略将总测试时间最小化，但是在特定特性的代码开发期间，或者在调试期间，我们可能不希望运行整个测试集。对于调试和代码开发，我们只需要能够运行选定的测试子集。在本示例中，我们将实现这一策略。

**准备工作**

在这个例子中，我们假设总共有六个测试：前三个测试比较短，名称分别为feature-a、feature-b和feature-c，还有三个长测试，名称分别是feature-d、benchmark-a和benchmark-b。这个示例中，我们可以用Python脚本表示这些测试，可以在其中调整休眠时间:

.. code-block:: python

  import sys
  import time
  # wait for 0.1 seconds
  time.sleep(0.1)
  # finally report success
  sys.exit(0)

具体实施

以下是我们CMakeLists.txt文件内容的详细内容:

1 CMakeLists.txt中，定义了六个测试:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name
  project(recipe-09 LANGUAGES NONE)
  # detect python
  find_package(PythonInterp REQUIRED)
  # define tests
  enable_testing()
  add_test(
    NAME feature-a
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/feature-a.py
    )
  add_test(
    NAME feature-b
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/feature-b.py
    )
  add_test(
    NAME feature-c
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/feature-c.py
    )
  add_test(
    NAME feature-d
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/feature-d.py
    )
  add_test(
    NAME benchmark-a
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/benchmark-a.py
    )
  add_test(
    NAME benchmark-b
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/benchmark-b.py
    )

2 此外，我们给较短的测试贴上quick的标签，给较长的测试贴上long的标签:

.. code-block:: cmake

  set_tests_properties(
    feature-a
    feature-b
    feature-c
    PROPERTIES
        LABELS "quick"
    )
  set_tests_properties(
    feature-d
    benchmark-a
    benchmark-b
    PROPERTIES
        LABELS "long"
    )

3 我们现在可以运行测试集了，如下:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ ctest
  Start 1: feature-a
  1/6 Test #1: feature-a ........................ Passed 0.11 sec
  Start 2: feature-b
  2/6 Test #2: feature-b ........................ Passed 0.11 sec
  Start 3: feature-c
  3/6 Test #3: feature-c ........................ Passed 0.11 sec
  Start 4: feature-d
  4/6 Test #4: feature-d ........................ Passed 0.51 sec
  Start 5: benchmark-a
  5/6 Test #5: benchmark-a ...................... Passed 0.51 sec
  Start 6: benchmark-b
  6/6 Test #6: benchmark-b ...................... Passed 0.51 sec
  100% tests passed, 0 tests failed out of 6
  Label Time Summary:
  long = 1.54 sec*proc (3 tests)
  quick = 0.33 sec*proc (3 tests)
  Total Test time (real) = 1.87 sec

**工作原理**

现在每个测试都有一个名称和一个标签。CMake中所有的测试都是有编号的，所以它们也带有唯一编号。定义了测试标签之后，我们现在可以运行整个集合，或者根据它们的名称(使用正则表达式)、标签或编号运行测试。

按名称运行测试(运行所有具有名称匹配功能的测试):

.. code-block:: bash

  $ ctest -R feature
  Start 1: feature-a
  1/4 Test #1: feature-a ........................ Passed 0.11 sec
  Start 2: feature-b
  2/4 Test #2: feature-b ........................ Passed 0.11 sec
  Start 3: feature-c
  3/4 Test #3: feature-c ........................ Passed 0.11 sec
  Start 4: feature-d
  4/4 Test #4: feature-d ........................ Passed 0.51 sec
  100% tests passed, 0 tests failed out of 4

按照标签运行测试(运行所有的长测试):

.. code-block:: bash

  $ ctest -L long
  Start 4: feature-d
  1/3 Test #4: feature-d ........................ Passed 0.51 sec
  Start 5: benchmark-a
  2/3 Test #5: benchmark-a ...................... Passed 0.51 sec
  Start 6: benchmark-b
  3/3 Test #6: benchmark-b ...................... Passed 0.51 sec
  100% tests passed, 0 tests failed out of 3

根据数量运行测试(运行测试2到4)产生的结果是:

.. code-block:: bash

  $ ctest -I 2,4
  Start 2: feature-b
  1/3 Test #2: feature-b ........................ Passed 0.11 sec
  Start 3: feature-c
  2/3 Test #3: feature-c ........................ Passed 0.11 sec
  Start 4: feature-d
  3/3 Test #4: feature-d ........................ Passed 0.51 sec
  100% tests passed, 0 tests failed out of 3

**更多信息**

尝试使用$ ctest --help，将看到有大量的选项可供用来定制测试。

4.10 使用测试固件
--------------------------

.. NOTE::
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-04/recipe-10 中找到。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

这个示例的灵感来自于Craig Scott，我们建议读者也参考相应的博客文章来了解更多的背景知识，https://crascit.com/2016/10/18/test-fixtures-withcmake-ctest/ ，此示例的动机是演示如何使用测试固件。这对于更复杂的测试非常有用，这些测试需要在测试运行前进行设置，以及在测试完成后执行清理操作(例如：创建示例数据库、设置连接、断开连接、清理测试数据库等等)。我们需要运行一个设置或清理操作的测试，并能够以一种可预测和健壮的方式自动触发这些步骤，而不需要引入代码重复。这些设置和清理步骤可以委托给测试框架(例如Google Test或Catch2)，我们在这里将演示如何在CMake级别实现测试固件。

**准备工作**

我们将准备4个Python脚本，并将它们放在test目录下:setup.py、features-a.py、features-b.py和clean-up.py。

**具体实施**

1 我们从CMakeLists.txt结构开始，附加一些步骤如下:

基础CMake语句:

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name
  project(recipe-10 LANGUAGES NONE)
  # detect python
  find_package(PythonInterp REQUIRED)
  # define tests
  enable_testing()

2 然后，定义了4个测试步骤，并将它们绑定到一个固件上:

.. code-block:: cmake

  add_test(
    NAME setup
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/setup.py
    )
  set_tests_properties(
    setup
    PROPERTIES
        FIXTURES_SETUP my-fixture
    )
  add_test(
    NAME feature-a
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/feature-a.py
    )
  add_test(
    NAME feature-b
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/feature-b.py
    )
  set_tests_properties(
    feature-a
    feature-b
    PROPERTIES
        FIXTURES_REQUIRED my-fixture
    )
  add_test(
    NAME cleanup
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/cleanup.py
    )
  set_tests_properties(
    cleanup
    PROPERTIES
        FIXTURES_CLEANUP my-fixture
    )

3 运行整个集合，如下面的输出所示:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ ctest
  Start 1: setup
  1/4 Test #1: setup ............................ Passed 0.01 sec
  Start 2: feature-a
  2/4 Test #2: feature-a ........................ Passed 0.01 sec
  Start 3: feature-b
  3/4 Test #3: feature-b ........................ Passed 0.00 sec
  Start 4: cleanup
  4/4 Test #4: cleanup .......................... Passed 0.01 sec
  100% tests passed, 0 tests failed out of 4

4 然而，当我们试图单独运行测试特性时。它正确地调用设置步骤和清理步骤:

.. code-block:: bash

  $ ctest -R feature-a
  Start 1: setup
  1/3 Test #1: setup ............................ Passed 0.01 sec
  Start 2: feature-a
  2/3 Test #2: feature-a ........................ Passed 0.00 sec
  Start 4: cleanup
  3/3 Test #4: cleanup .......................... Passed 0.01 sec
  100% tests passed, 0 tests failed out of 3

**工作原理**

在本例中，我们定义了一个文本固件，并将其称为my-fixture。我们为安装测试提供了FIXTURES_SETUP属性，并为清理测试了FIXTURES_CLEANUP属性，并且使用FIXTURES_REQUIRED，我们确保测试feature-a和feature-b都需要安装和清理步骤才能运行。将它们绑定在一起，可以确保在定义良好的状态下，进入和离开相应的步骤。

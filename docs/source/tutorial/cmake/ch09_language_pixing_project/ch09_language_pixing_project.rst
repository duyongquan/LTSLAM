.. highlight:: c++

.. default-domain:: cpp

==========================
第9章 语言混合项目
==========================

9.1 使用C/C++库构建Fortran项目
----------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-9/recipe-01 中找到，
  其中有两个示例：一个是Fortran与C的混例，另一个是Fortran和C++的混例。该示例在CMake 3.5版(或更高版本)中是有效的，
  并且已经在GNU/Linux、macOS和Windows上进行过测试。

Fortran作为高性能计算语言有着悠久的历史。目前，许多线性代数库仍然使用Fortran语言编写，许多大型的数字处理包也保持与过去几十年的代码兼容。
而Fortran提出了一个很自然的语法处理数值数组，它缺乏与操作系统交互，所以为了编程的通用性，需要一个互操作性层(使用C实现)，才发布了Fortran 2003标准。
本示例将展示如何用C系统库和自定义C代码来对接Fortran代码。

**具体实施**

我们有4个CMakeLists.txt实例要查看——根目录下1个，子目录下3个。让我们从根目录的CMakeLists.txt开始:

1 声明一个Fortran和C的混合语言项目:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES Fortran C)

2 CMake将静态库和动态库保存在build目录下的lib目录中。可执行文件保存在bin目录下，Fortran编译模块文件保存在modules目录下:

.. code-block:: cmake

  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
  set(CMAKE_Fortran_MODULE_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/modules)

3 接下来，我们进入第一个子CMakeLists.txt，添加src子目录:

.. code-block:: cmake

  add_subdirectory(src)

4 src/CMakeLists.txt文件添加了两个子目录:

.. code-block:: cmake

  add_subdirectory(interfaces)
  add_subdirectory(utils)

在interfaces子目录中，我们将执行以下操作:

1 包括FortranCInterface.cmak模块，并验证C和Fortran编译器可以正确地交互:

.. code-block:: cmake

  include(FortranCInterface)
  FortranCInterface_VERIFY()

2 接下来，我们找到Backtrace系统库，因为我们想在Fortran代码中使用它:

.. code-block:: cmake

  find_package(Backtrace REQUIRED)

3 然后，创建一个共享库目标，其中包含Backtrace包装器、随机数生成器，以及Fortran包装器的源文件:

.. code-block:: cmake

  add_library(bt-randomgen-wrap SHARED "")
  target_sources(bt-randomgen-wrap
    PRIVATE
      interface_backtrace.f90
      interface_randomgen.f90
      randomgen.c
    )

4 我们还为新生成的库目标设置了链接库。使用PUBLIC属性，以便连接到其他目标时，能正确地看到依赖关系:

.. code-block:: cmake

  target_link_libraries(bt-randomgen-wrap
    PUBLIC
        ${Backtrace_LIBRARIES}
    )

5 utils子目录中，还有一个CMakeLists.txt，其只有一单行程序：我们创建一个新的库目标，子目录中的源文件将被编译到这个目标库中。并与这个目标没有依赖关系:

.. code-block:: cmake

  add_library(utils SHARED util_strings.f90)

回到src/CMakeLists.txt:

1 使用bt-randomgen-example.f90添加一个可执行目标:

.. code-block:: cmake

  add_executable(bt-randomgen-example bt-randomgen-example.f90)

2 最后，将在子CMakeLists.txt中生成的库目标，并链接到可执行目标:

.. code-block:: cmake

  target_link_libraries(bt-randomgen-example
    PRIVATE
        bt-randomgen-wrap
        utils
    )


9.2 使用Fortran库构建C/C++项目
----------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-9/recipe-02 中找到，
  其中有一个示例：一个是C++、C和Fortran的混例。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

第3章第4节，展示了如何检测Fortran编写的BLAS和LAPACK线性代数库，以及如何在C++代码中使用它们。这里，将重新讨论这个方式，但这次的角度有所不同：较少地关注检测外部库，
会更深入地讨论混合C++和Fortran的方面，以及名称混乱的问题。

**具体实施**

这个项目混合了C++(作为该示例的主程序语言)和C(封装Fortran子例程所需的语言)。在根目录下的CMakeLists.txt文件中，我们需要做以下操作:

1 声明一个混合语言项目，并选择C++标准：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-02 LANGUAGES CXX C Fortran)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 使用GNUInstallDirs模块来设置CMake将静态和动态库，以及可执行文件保存的标准目录。我们还指示CMake将Fortran编译的模块文件放在modules目录下:

.. code-block:: cmake

  include(GNUInstallDirs)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
  set(CMAKE_Fortran_MODULE_DIRECTORY ${PROJECT_BINARY_DIR}/modules)

3 然后，进入下一个子目录:

.. code-block:: cmake

  add_subdirectory(src)

子文件src/CMakeLists.txt添加了另一个目录math，其中包含线性代数包装器。在src/math/CMakeLists.txt中，我们需要以下操作:

1 调用find_package来获取BLAS和LAPACK库的位置:

.. code-block:: cmake

  find_package(BLAS REQUIRED)
  find_package(LAPACK REQUIRED)

2 包含FortranCInterface.cmake模块，并验证Fortran、C和C++编译器是否兼容:

.. code-block:: cmake

  include(FortranCInterface)
  FortranCInterface_VERIFY(CXX)

3 我们还需要生成预处理器宏来处理BLAS和LAPACK子例程的名称问题。同样，FortranCInterface通过在当前构建目录中生成一个名为fc_mangl.h的头文件来提供协助:

.. code-block:: cmake

  FortranCInterface_HEADER(
    fc_mangle.h
    MACRO_NAMESPACE "FC_"
    SYMBOLS DSCAL DGESV
    )

4 接下来，添加了一个库，其中包含BLAS和LAPACK包装器的源代码。我们还指定要找到头文件和库的目录。注意PUBLIC属性，它允许其他依赖于math的目标正确地获得它们的依赖关系:

.. code-block:: cmake

  add_library(math "")
  target_sources(math
    PRIVATE
      CxxBLAS.cpp
      CxxLAPACK.cpp
    )
  target_include_directories(math
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
    )
  target_link_libraries(math
    PUBLIC
        ${LAPACK_LIBRARIES}
    )

5 回到src/CMakeLists.txt，我们最终添加了一个可执行目标，并将其链接到BLAS/LAPACK包装器的数学库:

.. code-block:: cmake

  add_executable(linear-algebra "")
  target_sources(linear-algebra
    PRIVATE
        linear-algebra.cpp
    )
  target_link_libraries(linear- algebra
    PRIVATE
        math
    )


9.3 使用Cython构建C++和Python项目
----------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-9/recipe-03 中找到，其中有一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

Cython是一个静态编译器，它允许为Python编写C扩展。Cython是一个非常强大的工具，使用Cython编程语言(基于Pyrex)。
Cython的一个典型用例是加快Python代码的速度，它也可以用于通过Cython层使Python与C(++)接口对接。本示例中，我们将重点介绍后一种用例，
并演示如何在CMake的帮助下使用Cython与C(++)和Python进行对接。


**具体实施**

如何生成Python接口:

1 CMakeLists.txt定义CMake依赖项、项目名称和语言:

.. code-block:: cmake

  # define minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name and supported language
  project(recipe-03 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 Windows上，最好不要保留未定义的构建类型，这样我们就可以将该项目的构建类型与Python环境的构建类型相匹配。这里我们默认为Release类型:

.. code-block:: cmake

  if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()

3 在示例中，还需要Python解释器:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)

4 下面的CMake代码将构建Python模块:

.. code-block:: cmake

  # directory cointaining UseCython.cmake and FindCython.cmake
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-cython)
  # this defines cython_add_module
  include(UseCython)
  # tells UseCython to compile this file as a c++ file
  set_source_files_properties(account.pyx PROPERTIES CYTHON_IS_CXX TRUE)
  # create python module
  cython_add_module(account account.pyx account.cpp)
  # location of account.hpp
  target_include_directories(account
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

5 定义一个测试：

.. code-block:: bash

  # turn on testing
  enable_testing()
  # define test
  add_test(
    NAME
        python_test
    COMMAND
        ${CMAKE_COMMAND} -E env ACCOUNT_MODULE_PATH=$<TARGET_FILE_DIR:account>
        ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py
    )
  python_test执行test.py，这里进行一些存款和取款操作，并验证余额:

  import os
  import sys
  sys.path.append(os.getenv('ACCOUNT_MODULE_PATH'))
  from account import pyAccount as Account
  account1 = Account()
  account1.deposit(100.0)
  account1.deposit(100.0)
  account2 = Account()
  account2.deposit(200.0)
  account2.deposit(200.0)
  account1.withdraw(50.0)
  assert account1.get_balance() == 150.0
  assert account2.get_balance() == 400.0

6 有了这个，我们就可以配置、构建和测试代码了:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest
  Start 1: python_test
  1/1 Test #1: python_test ...................... Passed 0.03 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.03 sec


9.4 使用Boost.Python构建C++和Python项目
----------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-9/recipe-04 中找到，其中有一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

Boost库为C++代码提供了Python接口。本示例将展示如何在依赖于Boost的C++项目中使用CMake，之后将其作为Python模块发布。我们将重用前面的示例，
并尝试用Cython示例中的C++实现(account.cpp)进行交互。


**具体实施**

如何在C++项目中使用Boost.Python的步骤：

1 和之前一样，首先定义最低版本、项目名称、支持语言和默认构建类型:

.. code-block:: cmake

  # define minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name and supported language
  project(recipe-04 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # we default to Release build type
  if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()

2 本示例中，依赖Python和Boost库，以及使用Python进行测试。Boost.Python组件依赖于Boost版本和Python版本，因此需要对这两个组件的名称进行检测：

.. code-block:: cmake

  # for testing we will need the python interpreter
  find_package(PythonInterp REQUIRED)
  # we require python development headers
  find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} EXACT REQUIRED)
  # now search for the boost component
  # depending on the boost version it is called either python,
  # python2, python27, python3, python36, python37, ...
  list(
    APPEND _components
      python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}
      python${PYTHON_VERSION_MAJOR}
      python
    )
  set(_boost_component_found "")
  foreach(_component IN ITEMS ${_components})
    find_package(Boost COMPONENTS ${_component})
    if(Boost_FOUND)
        set(_boost_component_found ${_component})
        break()
    endif()
  endforeach()
  if(_boost_component_found STREQUAL "")
      message(FATAL_ERROR "No matching Boost.Python component found")
  endif()

3 使用以下命令，定义Python模块及其依赖项:

.. code-block:: cmake

  # create python module
  add_library(account
    MODULE
        account.cpp
    )
  target_link_libraries(account
    PUBLIC
        Boost::${_boost_component_found}
    ${PYTHON_LIBRARIES}
    )
  target_include_directories(account
    PRIVATE
        ${PYTHON_INCLUDE_DIRS}
    )
  # prevent cmake from creating a "lib" prefix
  set_target_properties(account
    PROPERTIES
        PREFIX ""
    )
  if(WIN32)
    # python will not import dll but expects pyd
    set_target_properties(account
      PROPERTIES
          SUFFIX ".pyd"
    )
  endif()

4 最后，定义了一个测试:

.. code-block:: cmake

  # turn on testing
  enable_testing()
  # define test
  add_test(
    NAME
        python_test
    COMMAND
        ${CMAKE_COMMAND} -E env ACCOUNT_MODULE_PATH=$<TARGET_FILE_DIR:account>
        ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test.py
    )

5 配置、编译和测试:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest
  Start 1: python_test
  1/1 Test #1: python_test ...................... Passed 0.10 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.11 sec


9.5 使用pybind11构建C++和Python项目
----------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-9/recipe-05 中找到，其中有一个C++示例。
  该示例在CMake 3.11版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

前面的示例中，我们使用Boost.Python与C(C++)接口。本示例中，我们将尝试使用pybind11将Python与C++接口。其实现利用了C++11的特性，
因此需要支持C++11的编译器。我们将演示在配置时如何获取pybind11依赖和构建我们的项目，包括一个使用FetchContent方法的Python接口，
我们在第4章第3节和第8章第4节中有过讨论。在第11章第2节时，会通过PyPI发布一个用CMake/pybind11构建的C++/Python项目。届时将重新讨论这个例子，
并展示如何打包它，使它可以用pip安装。

**具体实施**

让我们详细分析一下这个项目中，各个CMakeLists.txt文件的内容:

1 主CMakeLists.txt文件:

.. code-block:: cmake

  # define minimum cmake version
  cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
  # project name and supported language
  project(recipe-05 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 这个文件中，查询了用于测试的Python解释器:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)

3 然后，包含account子目录:

.. code-block:: cmake

  add_subdirectory(account)

4 定义单元测试:

.. code-block:: cmake

  # turn on testing
  enable_testing()
  # define test
  add_test(
    NAME
      python_test
    COMMAND
      ${CMAKE_COMMAND} -E env ACCOUNT_MODULE_PATH=$<TARGET_FILE_DIR:account>
      ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/account/test.py
  )

5 account/CMakeLists.txt中，在配置时获取pybind11的源码：

.. code-block:: cmake

  include(FetchContent)
  FetchContent_Declare(
    pybind11_sources
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.2
    )
  FetchContent_GetProperties(pybind11_sources)
  if(NOT pybind11_sources_POPULATED)
    FetchContent_Populate(pybind11_sources)
    add_subdirectory(
      ${pybind11_sources_SOURCE_DIR}
      ${pybind11_sources_BINARY_DIR}
      )
  endif()

6 最后，定义Python模块。再次使用模块选项add_library。并将库目标的前缀和后缀属性设置为PYTHON_MODULE_PREFIX和PYTHON_MODULE_EXTENSION，
这两个值由pybind11适当地推断出来:

.. code-block:: cmake

  add_library(account
    MODULE
      account.cpp
    )
  target_link_libraries(account
    PUBLIC
      pybind11::module
    )
  set_target_properties(account
    PROPERTIES
      PREFIX "${PYTHON_MODULE_PREFIX}"
      SUFFIX "${PYTHON_MODULE_EXTENSION}"
    )

7 进行测试：

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest
  Start 1: python_test
  1/1 Test #1: python_test ...................... Passed 0.04 sec
  100% tests passed, 0 tests failed out of 1
  Total Test time (real) = 0.04 sec


9.6 使用Python CFFI混合C，C++，Fortran和Python
-----------------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-9/recipe-06 中找到，其中有一个C++示例和一个Fortran示例。
  该示例在CMake 3.11版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

前面的三个示例中，我们使用Cython、Boost.Python和pybind11作为连接Python和C++的工具。之前的示例中，主要连接的是C++接口。然而，
可能会遇到这样的情况：将Python与Fortran或其他语言进行接口。

本示例中，我们将使用Python C的外部函数接口(CFFI，参见https://cffi.readthedocs.io)。由于C是通用语言，大多数编程语言(包括Fortran)都能够与C接口进行通信，
所以Python CFFI是将Python与大量语言结合在一起的工具。Python CFFI的特性是，生成简单且非侵入性的C接口，这意味着它既不限制语言特性中的Python层，
也不会对C层以下的代码有任何限制。

本示例中，将使用前面示例的银行帐户示例，通过C接口将Python CFFI应用于Python和C++。我们的目标是实现一个上下文感知的接口。接口中，我们可以实例化几个银行帐户，
每个帐户都带有其内部状态。我们将通过讨论如何使用Python CFFI来连接Python和Fortran来结束本教程。

第11章第3节中，通过PyPI分发一个用CMake/CFFI构建的C/Fortran/Python项目，届时我们将重新讨论这个例子，并展示如何打包它，使它可以用pip安装。


**具体实施**

现在使用CMake来组合这些文件，形成一个Python模块:

1 主CMakeLists.txt文件包含一个头文件。此外，根据GNU标准，设置编译库的位置:

.. code-block:: cmake

  # define minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name and supported language
  project(recipe-06 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # specify where to place libraries
  include(GNUInstallDirs)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
  ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

2 第二步，是在account子目录下包含接口和实现的定义：

.. code-block:: cmake

  # interface and sources
  add_subdirectory(account)

3 主CMakeLists.txt文件以测试定义(需要Python解释器)结束：

.. code-block:: cmake

  # turn on testing
  enable_testing()
  # require python
  find_package(PythonInterp REQUIRED)
  # define test
  add_test(
    NAME
      python_test
    COMMAND
      ${CMAKE_COMMAND} -E env ACCOUNT_MODULE_PATH=${CMAKE_CURRENT_SOURCE_DIR}
                          ACCOUNT_HEADER_FILE=${CMAKE_CURRENT_SOURCE_DIR}/account/account.h
                          ACCOUNT_LIBRARY_FILE=$<TARGET_FILE:account>
    ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/account/test.py
  )

4 account/CMakeLists.txt中定义了动态库目标：

.. code-block:: cmake

  add_library(account
    SHARED
      plementation/c_cpp_interface.cpp
      implementation/cpp_implementation.cpp
    )
  target_include_directories(account
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
    )

5 导出一个可移植的头文件:

.. code-block:: cmake

  include(GenerateExportHeader)
  generate_export_header(account
    BASE_NAME account
    )

6 使用Python-C接口进行对接:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ctest
  Start 1: python_test
  1/1 Test #1: python_test ...................... Passed 0.14 sec
  100% tests passed, 0 tests failed out of 1


.. highlight:: c++

.. default-domain:: cpp

==========================
第2章 检测环境
==========================

本章的主要内容有：

* 检测操作系统
* 处理与平台相关的源码
* 处理与编译器相关的源码
* 检测处理器体系结构
* 检测处理器指令集
* 为Eigen库使能向量化

尽管CMake跨平台，但有时源代码并不是完全可移植(例如：当使用依赖于供应商的扩展时)，我们努力使源代码能够跨平台、操作系统和编译器。
这个过程中会发现，有必要根据平台不同的方式配置和/或构建代码。这对于历史代码或交叉编译尤其重要，我们将在第13章中讨论这个主题。
了解处理器指令集也有助于优化特定目标平台的性能。本章会介绍，检测环境的方法，并给出建议。

2.1 检测操作系统
----------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-02/recipe-01 中找到。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

CMake是一组跨平台工具。不过，了解操作系统(OS)上执行配置或构建步骤也很重要。从而与操作系统相关的CMake代码，会根据操作系统启用条件编译，
或者在可用或必要时使用特定于编译器的扩展。本示例中，我们将通过一个不需要编译任何源代码的示例，演示如何使用CMake检测操作系统。为了简单起见，
我们只考虑配置过程。

**具体实施**

我们将用一个非常简单的CMakeLists.txt进行演示:

1 首先，定义CMake最低版本和项目名称。请注意，语言是NONE:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES NONE)

2 然后，根据检测到的操作系统信息打印消息:

.. code-block:: cmake

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
      message(STATUS "Configuring on/for Linux")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      message(STATUS "Configuring on/for macOS")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
      message(STATUS "Configuring on/for Windows")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "AIX")
      message(STATUS "Configuring on/for IBM AIX")
  else()
      message(STATUS "Configuring on/for ${CMAKE_SYSTEM_NAME}")
  endif()

测试之前，检查前面的代码块，并考虑相应系统上的具体行为。

3 现在，测试配置项目:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..

4 关于CMake输出，这里有一行很有趣——在Linux系统上(在其他系统上，输出会不同):

.. code-block:: bash

  -- Configuring on/for Linux

**工作原理**

CMake为目标操作系统定义了CMAKE_SYSTEM_NAME，因此不需要使用定制命令、工具或脚本来查询此信息。然后，可以使用此变量的值实现特定于操作系统的
条件和解决方案。在具有uname命令的系统上，将此变量设置为uname -s的输出。该变量在macOS上设置为“Darwin”。在Linux和Windows上，
它分别计算为“Linux”和“Windows”。我们了解了如何在特定的操作系统上执行特定的CMake代码。当然，应该尽量减少这种定制化行为，以便简化迁移到新平台的过程。

2.2 处理与平台相关的源代码
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-02/recipe-02 中找到，包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

理想情况下，应该避免依赖于平台的源代码，但是有时我们没有选择，特别是当要求配置和编译不是自己编写的代码时。本示例中，将演示如何使用CMake根据操作系统编译源代码。

**准备工作**

修改hello-world.cpp示例代码，将第1章第1节的例子进行修改:

.. code-block:: c++

  #include <cstdlib>
  #include <iostream>
  #include <string>
  std::string say_hello() {
  #ifdef IS_WINDOWS
    return std::string("Hello from Windows!");
  #elif IS_LINUX
    return std::string("Hello from Linux!");
  #elif IS_MACOS
    return std::string("Hello from macOS!");
  #else
    return std::string("Hello from an unknown system!");
  #endif
  }
  int main() {
    std::cout << say_hello() << std::endl;
    return EXIT_SUCCESS;
  }

**具体实施**

完成一个CMakeLists.txt实例，使我们能够基于目标操作系统有条件地编译源代码：

1 首先，设置了CMake最低版本、项目名称和支持的语言:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-02 LANGUAGES CXX)

2 然后，定义可执行文件及其对应的源文件:

.. code-block:: cmake

  add_executable(hello-world hello-world.cpp)

3 通过定义以下目标编译定义，让预处理器知道系统名称:

.. code-block:: cmake

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(hello-world PUBLIC "IS_LINUX")
  endif()
  if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    target_compile_definitions(hello-world PUBLIC "IS_MACOS")
  endif()
  if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_definitions(hello-world PUBLIC "IS_WINDOWS")
  endif()

继续之前，先检查前面的表达式，并考虑在不同系统上有哪些行为。

4 现在，准备测试它，并配置项目:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./hello-world
  Hello from Linux!

Windows系统上，将看到来自Windows的Hello。其他操作系统将产生不同的输出。

**工作原理**

hello-world.cpp示例中，有趣的部分是基于预处理器定义IS_WINDOWS、IS_LINUX或IS_MACOS的条件编译:

.. code-block:: c++

  std::string say_hello() {
  #ifdef IS_WINDOWS
    return std::string("Hello from Windows!");
  #elif IS_LINUX
    return std::string("Hello from Linux!");
  #elif IS_MACOS
    return std::string("Hello from macOS!");
  #else
    return std::string("Hello from an unknown system!");
  #endif
  }

这些定义在CMakeLists.txt中配置时定义，通过使用target_compile_definition在预处理阶段使用。可以不重复if-endif语句，以更紧凑的表达式实现，
我们将在下一个示例中演示这种重构方式。也可以把if-endif语句加入到一个if-else-else-endif语句中。这个阶段，可以使用add_definitions(-DIS_LINUX)
来设置定义(当然，可以根据平台调整定义)，而不是使用target_compile_definition。使用add_definitions的缺点是，会修改编译整个项目的定义，
而target_compile_definitions给我们机会，将定义限制于一个特定的目标，以及通过PRIVATE|PUBLIC|INTERFACE限定符，限制这些定义可见性。
第1章的第8节，对这些限定符有详细的说明:


* PRIVATE，编译定义将只应用于给定的目标，而不应用于相关的其他目标。
* INTERFACE，对给定目标的编译定义将只应用于使用它的目标。
* PUBLIC，编译定义将应用于给定的目标和使用它的所有其他目标。

2.3 处理与编译器相关的源代码
-------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-02/recipe-03 中找到，包含一个C++和Fortran示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

这个方法与前面的方法类似，我们将使用CMake来编译依赖于环境的条件源代码：本例将依赖于编译器。为了可移植性，我们尽量避免去编写新代码，但遇到有依赖的情况我们也要去解决，特别是当使用历史代码或处理编译器依赖工具，如sanitizers。从这一章和前一章的示例中，我们已经掌握了实现这一目标的所有方法。尽管如此，讨论与编译器相关的源代码的处理问题还是很有用的，这样我们将有机会从另一方面了解CMake。

**准备工作**

本示例中，我们将从C++中的一个示例开始，稍后我们将演示一个Fortran示例，并尝试重构和简化CMake代码。

看一下hello-world.cpp源代码:

.. code-block:: c++

  #include <cstdlib>
  #include <iostream>
  #include <string>
  std::string say_hello() {
  #ifdef IS_INTEL_CXX_COMPILER
    // only compiled when Intel compiler is selected
    // such compiler will not compile the other branches
    return std::string("Hello Intel compiler!");
  #elif IS_GNU_CXX_COMPILER
    // only compiled when GNU compiler is selected
    // such compiler will not compile the other branches
    return std::string("Hello GNU compiler!");
  #elif IS_PGI_CXX_COMPILER
    // etc.
    return std::string("Hello PGI compiler!");
  #elif IS_XL_CXX_COMPILER
    return std::string("Hello XL compiler!");
  #else
    return std::string("Hello unknown compiler - have we met before?");
  #endif
  }
  int main() {
    std::cout << say_hello() << std::endl;
    std::cout << "compiler name is " COMPILER_NAME << std::endl;
    return EXIT_SUCCESS;
  }
  Fortran示例(hello-world.F90):

  program hello
    implicit none
  #ifdef IS_Intel_FORTRAN_COMPILER
    print *, 'Hello Intel compiler!'
  #elif IS_GNU_FORTRAN_COMPILER
    print *, 'Hello GNU compiler!'
  #elif IS_PGI_FORTRAN_COMPILER
    print *, 'Hello PGI compiler!'
  #elif IS_XL_FORTRAN_COMPILER
    print *, 'Hello XL compiler!'
  #else
    print *, 'Hello unknown compiler - have we met before?'
  #endif
  end program

**具体实施**

我们将从C++的例子开始，然后再看Fortran的例子:

1 CMakeLists.txt文件中，定义了CMake最低版本、项目名称和支持的语言:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-03 LANGUAGES CXX)

2 然后，定义可执行目标及其对应的源文件:

.. code-block:: cmake

  add_executable(hello-world hello-world.cpp)

3 通过定义以下目标编译定义，让预处理器了解编译器的名称和供应商:

.. code-block:: cmake

  target_compile_definitions(hello-world PUBLIC "COMPILER_NAME=\"${CMAKE_CXX_COMPILER_ID}\"")
  if(CMAKE_CXX_COMPILER_ID MATCHES Intel)
    target_compile_definitions(hello-world PUBLIC "IS_INTEL_CXX_COMPILER")
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    target_compile_definitions(hello-world PUBLIC "IS_GNU_CXX_COMPILER")
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES PGI)
    target_compile_definitions(hello-world PUBLIC "IS_PGI_CXX_COMPILER")
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES XL)
    target_compile_definitions(hello-world PUBLIC "IS_XL_CXX_COMPILER")
  endif()

4 现在我们已经可以预测结果了:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./hello-world
  Hello GNU compiler!

使用不同的编译器，此示例代码将打印不同的问候语。

前一个示例的CMakeLists.txt文件中的if语句似乎是重复的，我们不喜欢重复的语句。能更简洁地表达吗？当然可以！为此，让我们再来看看Fortran示例。

Fortran例子的CMakeLists.txt文件中，我们需要做以下工作:

1 需要使Fortran语言:

.. code-block:: cmake

  project(recipe-03 LANGUAGES Fortran)

2 然后，定义可执行文件及其对应的源文件。在本例中，使用大写.F90后缀:

.. code-block:: cmake

  add_executable(hello-world hello-world.F90)

3 我们通过定义下面的目标编译定义，让预处理器非常清楚地了解编译器:

.. code-block:: cmake

  target_compile_definitions(hello-world
    PUBLIC "IS_${CMAKE_Fortran_COMPILER_ID}_FORTRAN_COMPILER"
    )

其余行为与C++示例相同。

**工作原理**

CMakeLists.txt会在配置时，进行预处理定义，并传递给预处理器。Fortran示例包含非常紧凑的表达式，我们使用CMAKE_Fortran_COMPILER_ID变量，
通过target_compile_definition使用构造预处理器进行预处理定义。为了适应这种情况，我们必须将”Intel”从IS_INTEL_CXX_COMPILER更改为
IS_Intel_FORTRAN_COMPILER。通过使用相应的CMAKE_C_COMPILER_ID和CMAKE_CXX_COMPILER_ID变量，我们可以在C或C++中实现相同的效果。
但是，请注意，CMAKE_<LANG>_COMPILER_ID不能保证为所有编译器或语言都定义。


2.4 检测处理器体系结构
----------------------

.. NOTE:: 

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-02/recipe-04 中找到，包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

19世纪70年代，出现的64位整数运算和本世纪初出现的用于个人计算机的64位寻址，扩大了内存寻址范围，开发商投入了大量资源来移植为32位体系结构硬编码，
以支持64位寻址。许多博客文章，如 https://www.viva64.com/en/a/0004/ ，致力于讨论将C++代码移植到64位平台中的典型问题和解决方案。虽然，
避免显式硬编码的方式非常明智，但需要在使用CMake配置的代码中适应硬编码限制。本示例中，我们会来讨论检测主机处理器体系结构的选项。

**准备工作**

我们以下面的arch-dependent.cpp代码为例：

.. code-block:: c++

  #include <cstdlib>
  #include <iostream>
  #include <string>
  #define STRINGIFY(x) #x
  #define TOSTRING(x) STRINGIFY(x)
  std::string say_hello()
  {
    std::string arch_info(TOSTRING(ARCHITECTURE));
    arch_info += std::string(" architecture. ");
  #ifdef IS_32_BIT_ARCH
    return arch_info + std::string("Compiled on a 32 bit host processor.");
  #elif IS_64_BIT_ARCH
    return arch_info + std::string("Compiled on a 64 bit host processor.");
  #else
    return arch_info + std::string("Neither 32 nor 64 bit, puzzling ...");
  #endif
  }
  int main()
  {
    std::cout << say_hello() << std::endl;
    return EXIT_SUCCESS;
  }

**具体实施**

CMakeLists.txt文件中，我们需要以下内容:

1 首先，定义可执行文件及其源文件依赖关系:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-04 LANGUAGES CXX)
  add_executable(arch-dependent arch-dependent.cpp)

2 检查空指针类型的大小。CMake的CMAKE_SIZEOF_VOID_P变量会告诉我们CPU是32位还是64位。我们通过状态消息让用户知道检测到的大小，并设置预处理器定义:

.. code-block:: cmake

  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    target_compile_definitions(arch-dependent PUBLIC "IS_64_BIT_ARCH")
    message(STATUS "Target is 64 bits")
  else()
    target_compile_definitions(arch-dependent PUBLIC "IS_32_BIT_ARCH")
    message(STATUS "Target is 32 bits")
  endif()

3 通过定义以下目标编译定义，让预处理器了解主机处理器架构，同时在配置过程中打印状态消息:

.. code-block:: cmake

  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "i386")
      message(STATUS "i386 architecture detected")
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "i686")
      message(STATUS "i686 architecture detected")
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
      message(STATUS "x86_64 architecture detected")
  else()
      message(STATUS "host processor architecture is unknown")
  endif()
  target_compile_definitions(arch-dependent
    PUBLIC "ARCHITECTURE=${CMAKE_HOST_SYSTEM_PROCESSOR}"
    )

4 配置项目，并注意状态消息(打印出的信息可能会发生变化):

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  ...
  -- Target is 64 bits
  -- x86_64 architecture detected
  ...

5 最后，构建并执行代码(实际输出将取决于处理器架构):

.. code-block:: bash

  $ cmake --build .
  $ ./arch-dependent
  x86_64 architecture. Compiled on a 64 bit host processor.

**工作原理**

CMake定义了CMAKE_HOST_SYSTEM_PROCESSOR变量，以包含当前运行的处理器的名称。可以设置为“i386”、“i686”、“x86_64”、“AMD64”等等，
当然，这取决于当前的CPU。CMAKE_SIZEOF_VOID_P为void指针的大小。我们可以在CMake配置时进行查询，以便修改目标或目标编译定义。
可以基于检测到的主机处理器体系结构，使用预处理器定义，确定需要编译的分支源代码。正如在前面的示例中所讨论的，编写新代码时应该避免这种依赖，
但在处理遗留代码或交叉编译时，这种依赖是有用的，交叉编译会在第13章进行讨论。

2.5 检测处理器指令集
----------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-02/recipe-05 中找到，包含一个C++示例。
  该示例在CMake 3.10版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

本示例中，我们将讨论如何在CMake的帮助下检测主机处理器支持的指令集。这个功能是较新版本添加到CMake中的，需要CMake 3.10或更高版本。检测到的主机系统信息，可用于设置相应的编译器标志，或实现可选的源代码编译，或根据主机系统生成源代码。本示例中，我们的目标是检测主机系统信息，使用预处理器定义将其传递给C++源代码，并将信息打印到输出中。

**准备工作**

我们是C++源码(processor-info.cpp)如下所示：

.. code-block:: c++

  #include "config.h"
  #include <cstdlib>
  #include <iostream>
  int main()
  {
    std::cout << "Number of logical cores: "
              << NUMBER_OF_LOGICAL_CORES << std::endl;
    std::cout << "Number of physical cores: "
              << NUMBER_OF_PHYSICAL_CORES << std::endl;
    std::cout << "Total virtual memory in megabytes: "
              << TOTAL_VIRTUAL_MEMORY << std::endl;
    std::cout << "Available virtual memory in megabytes: "
              << AVAILABLE_VIRTUAL_MEMORY << std::endl;
    std::cout << "Total physical memory in megabytes: "
              << TOTAL_PHYSICAL_MEMORY << std::endl;
    std::cout << "Available physical memory in megabytes: "
              << AVAILABLE_PHYSICAL_MEMORY << std::endl;
    std::cout << "Processor is 64Bit: "
              << IS_64BIT << std::endl;
    std::cout << "Processor has floating point unit: "
              << HAS_FPU << std::endl;
    std::cout << "Processor supports MMX instructions: "
              << HAS_MMX << std::endl;
    std::cout << "Processor supports Ext. MMX instructions: "
              << HAS_MMX_PLUS << std::endl;
    std::cout << "Processor supports SSE instructions: "
              << HAS_SSE << std::endl;
    std::cout << "Processor supports SSE2 instructions: "
              << HAS_SSE2 << std::endl;
    std::cout << "Processor supports SSE FP instructions: "
              << HAS_SSE_FP << std::endl;
    std::cout << "Processor supports SSE MMX instructions: "
              << HAS_SSE_MMX << std::endl;
    std::cout << "Processor supports 3DNow instructions: "
              << HAS_AMD_3DNOW << std::endl;
    std::cout << "Processor supports 3DNow+ instructions: "
              << HAS_AMD_3DNOW_PLUS << std::endl;
    std::cout << "IA64 processor emulating x86 : "
              << HAS_IA64 << std::endl;
    std::cout << "OS name: "
              << OS_NAME << std::endl;
    std::cout << "OS sub-type: "
              << OS_RELEASE << std::endl;
    std::cout << "OS build ID: "
              << OS_VERSION << std::endl;
    std::cout << "OS platform: "
              << OS_PLATFORM << std::endl;
    return EXIT_SUCCESS;
  }

其包含config.h头文件，我们将使用config.h.in生成这个文件。config.h.in如下:

.. code-block:: c++

  #pragma once
  #define NUMBER_OF_LOGICAL_CORES @_NUMBER_OF_LOGICAL_CORES@
  #define NUMBER_OF_PHYSICAL_CORES @_NUMBER_OF_PHYSICAL_CORES@
  #define TOTAL_VIRTUAL_MEMORY @_TOTAL_VIRTUAL_MEMORY@
  #define AVAILABLE_VIRTUAL_MEMORY @_AVAILABLE_VIRTUAL_MEMORY@
  #define TOTAL_PHYSICAL_MEMORY @_TOTAL_PHYSICAL_MEMORY@
  #define AVAILABLE_PHYSICAL_MEMORY @_AVAILABLE_PHYSICAL_MEMORY@
  #define IS_64BIT @_IS_64BIT@
  #define HAS_FPU @_HAS_FPU@
  #define HAS_MMX @_HAS_MMX@
  #define HAS_MMX_PLUS @_HAS_MMX_PLUS@
  #define HAS_SSE @_HAS_SSE@
  #define HAS_SSE2 @_HAS_SSE2@
  #define HAS_SSE_FP @_HAS_SSE_FP@
  #define HAS_SSE_MMX @_HAS_SSE_MMX@
  #define HAS_AMD_3DNOW @_HAS_AMD_3DNOW@
  #define HAS_AMD_3DNOW_PLUS @_HAS_AMD_3DNOW_PLUS@
  #define HAS_IA64 @_HAS_IA64@
  #define OS_NAME "@_OS_NAME@"
  #define OS_RELEASE "@_OS_RELEASE@"
  #define OS_VERSION "@_OS_VERSION@"
  #define OS_PLATFORM "@_OS_PLATFORM@"

**如何实施**

我们将使用CMake为平台填充config.h中的定义，并将示例源文件编译为可执行文件:

1 首先，我们定义了CMake最低版本、项目名称和项目语言:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
  project(recipe-05 CXX)

2 然后，定义目标可执行文件及其源文件，并包括目录:

.. code-block:: cmake

  add_executable(processor-info "")
  target_sources(processor-info
    PRIVATE
        processor-info.cpp
    )
  target_include_directories(processor-info
    PRIVATE
        ${PROJECT_BINARY_DIR}
    )
3 继续查询主机系统的信息，获取一些关键字:

.. code-block:: cmake

  foreach(key
    IN ITEMS
      NUMBER_OF_LOGICAL_CORES
      NUMBER_OF_PHYSICAL_CORES
      TOTAL_VIRTUAL_MEMORY
      AVAILABLE_VIRTUAL_MEMORY
      TOTAL_PHYSICAL_MEMORY
      AVAILABLE_PHYSICAL_MEMORY
      IS_64BIT
      HAS_FPU
      HAS_MMX
      HAS_MMX_PLUS
      HAS_SSE
      HAS_SSE2
      HAS_SSE_FP
      HAS_SSE_MMX
      HAS_AMD_3DNOW
      HAS_AMD_3DNOW_PLUS
      HAS_IA64
      OS_NAME
      OS_RELEASE
      OS_VERSION
      OS_PLATFORM
    )
    cmake_host_system_information(RESULT _${key} QUERY ${key})
  endforeach()

4 定义了相应的变量后，配置config.h:

.. code-block:: cmake

  configure_file(config.h.in config.h @ONLY)

5 现在准备好配置、构建和测试项目:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./processor-info
  Number of logical cores: 4
  Number of physical cores: 2
  Total virtual memory in megabytes: 15258
  Available virtual memory in megabytes: 14678
  Total physical memory in megabytes: 7858
  Available physical memory in megabytes: 4072
  Processor is 64Bit: 1
  Processor has floating point unit: 1
  Processor supports MMX instructions: 1
  Processor supports Ext. MMX instructions: 0
  Processor supports SSE instructions: 1
  Processor supports SSE2 instructions: 1
  Processor supports SSE FP instructions: 0
  Processor supports SSE MMX instructions: 0
  Processor supports 3DNow instructions: 0
  Processor supports 3DNow+ instructions: 0
  IA64 processor emulating x86 : 0
  OS name: Linux
  OS sub-type: 4.16.7-1-ARCH
  OS build ID: #1 SMP PREEMPT Wed May 2 21:12:36 UTC 2018
  OS platform: x86_64

6 输出会随着处理器的不同而变化。

**工作原理**

CMakeLists.txt中的foreach循环会查询多个键值，并定义相应的变量。此示例的核心函数是cmake_host_system_information，
它查询运行CMake的主机系统的系统信息。本例中，我们对每个键使用了一个函数调用。然后，使用这些变量来配置config.h.in中的占位符，
输入并生成config.h。此配置使用configure_file命令完成。最后，config.h包含在processor-info.cpp中。编译后，它将把值打印到屏幕上。
我们将在第5章(配置时和构建时操作)和第6章(生成源代码)中重新讨论这种方法。


2.6 为Eigen库使能向量化
-------------------------

.. NOTE:: 
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-02/recipe-06 中找到，包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

处理器的向量功能，可以提高代码的性能。对于某些类型的运算来说尤为甚之，例如：线性代数。本示例将展示如何使能矢量化，以便使用线性代数的
Eigen C++库加速可执行文件。

**准备工作**

我们用Eigen C++模板库，用来进行线性代数计算，并展示如何设置编译器标志来启用向量化。这个示例的源代码linear-algebra.cpp文件:

.. code-block:: c++

  #include <chrono>
  #include <iostream>
  #include <Eigen/Dense>
  EIGEN_DONT_INLINE
  double simple_function(Eigen::VectorXd &va, Eigen::VectorXd &vb)
  {
    // this simple function computes the dot product of two vectors
    // of course it could be expressed more compactly
    double d = va.dot(vb);
    return d;
  }
  int main()
  {
    int len = 1000000;
    int num_repetitions = 100;
    // generate two random vectors
    Eigen::VectorXd va = Eigen::VectorXd::Random(len);
    Eigen::VectorXd vb = Eigen::VectorXd::Random(len);
    double result;
    auto start = std::chrono::system_clock::now();
    for (auto i = 0; i < num_repetitions; i++)
    {
      result = simple_function(va, vb);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed_seconds = end - start;
    std::cout << "result: " << result << std::endl;
    std::cout << "elapsed seconds: " << elapsed_seconds.count() << std::endl;
  }

我们期望向量化可以加快simple_function中的点积操作。

**如何实施**

根据Eigen库的文档，设置适当的编译器标志就足以生成向量化的代码。让我们看看CMakeLists.txt:

1 声明一个C++11项目:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-06 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  
2 使用Eigen库，我们需要在系统上找到它的头文件:

.. code-block:: cmake

  find_package(Eigen3 3.3 REQUIRED CONFIG)

3 CheckCXXCompilerFlag.cmake标准模块文件:

.. code-block:: cmake

  include(CheckCXXCompilerFlag)

4 检查-march=native编译器标志是否工作:

.. code-block:: cmake

  check_cxx_compiler_flag("-march=native" _march_native_works)

5 另一个选项-xHost编译器标志也开启:

.. code-block:: cmake

  check_cxx_compiler_flag("-xHost" _xhost_works)

6 设置了一个空变量_CXX_FLAGS，来保存刚才检查的两个编译器中找到的编译器标志。如果看到_march_native_works，
我们将_CXX_FLAGS设置为-march=native。如果看到_xhost_works，我们将_CXX_FLAGS设置为-xHost。
如果它们都不起作用，_CXX_FLAGS将为空，并禁用矢量化:

.. code-block:: cmake

  set(_CXX_FLAGS)
  if(_march_native_works)
      message(STATUS "Using processor's vector instructions (-march=native compiler flag set)")
      set(_CXX_FLAGS "-march=native")
  elseif(_xhost_works)
      message(STATUS "Using processor's vector instructions (-xHost compiler flag set)")
      set(_CXX_FLAGS "-xHost")
  else()
      message(STATUS "No suitable compiler flag found for vectorization")
  endif()

7 为了便于比较，我们还为未优化的版本定义了一个可执行目标，不使用优化标志:

.. code-block:: cmake

  add_executable(linear-algebra-unoptimized linear-algebra.cpp)
  target_link_libraries(linear-algebra-unoptimized
    PRIVATE
        Eigen3::Eigen
    )

8 此外，我们定义了一个优化版本:

.. code-block:: cmake

  add_executable(linear-algebra linear-algebra.cpp)
  target_compile_options(linear-algebra
    PRIVATE
        ${_CXX_FLAGS}
    )
  target_link_libraries(linear-algebra
    PRIVATE
        Eigen3::Eigen
    )

9 让我们比较一下这两个可执行文件——首先我们配置(在本例中，-march=native_works):

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  ...
  -- Performing Test _march_native_works
  -- Performing Test _march_native_works - Success
  -- Performing Test _xhost_works
  -- Performing Test _xhost_works - Failed
  -- Using processor's vector instructions (-march=native compiler flag set)
  ...


10 最后，让我们编译可执行文件，并比较运行时间:

.. code-block:: bash

  $ cmake --build .
  $ ./linear-algebra-unoptimized
  result: -261.505
  elapsed seconds: 1.97964
  $ ./linear-algebra
  result: -261.505
  elapsed seconds: 1.05048

**工作原理**

大多数处理器提供向量指令集，代码可以利用这些特性，获得更高的性能。由于线性代数运算可以从Eigen库中获得很好的加速，所以在使用Eigen库时，
就要考虑向量化。我们所要做的就是，指示编译器为我们检查处理器，并为当前体系结构生成本机指令。不同的编译器供应商会使用不同的标志来实现这一点：
GNU编译器使用-march=native标志来实现这一点，而Intel编译器使用-xHost标志。使用CheckCXXCompilerFlag.cmake模块提供的
check_cxx_compiler_flag函数进行编译器标志的检查:

.. code-block:: cmake

  check_cxx_compiler_flag("-march=native" _march_native_works)


这个函数接受两个参数:

* 第一个是要检查的编译器标志。
* 第二个是用来存储检查结果(true或false)的变量。如果检查为真，我们将工作标志添加到_CXX_FLAGS变量中，该变量将用于为可执行目标设置编译器标志。

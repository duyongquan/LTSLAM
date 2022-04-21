.. highlight:: c++

.. default-domain:: cpp

==========================
第10章 编写安装程序
==========================

10.1 安装项目
-------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-10/recipe-01 中找到，其中有一个C++示例和一个Fortran示例。
  该示例在CMake 3.6版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

第一个示例中，将介绍我们的小项目和一些基本概念，这些概念也将在后面的示例中使用。安装文件、库和可执行文件是一项非常基础的任务，但是也可能会带来一些缺陷。
我们将带您了解这些问题，并展示如何使用CMake有效地避开这些缺陷。

**准备工作**

第1章第3节的示例，几乎复用：只添加对UUID库的依赖。这个依赖是有条件的，如果没有找到UUID库，我们将通过预处理程序排除使用UUID库的代码。项目布局如下:

.. code-block:: bash

  .
  ├── CMakeLists.txt
  ├── src
  │    ├── CMakeLists.txt
  │    ├── hello-world.cpp
  │    ├── Message.cpp
  │    └── Message.hpp
  └── tests
      └── CMakeLists.txt

我们已经看到，有三个CMakeLists.txt，一个是主CMakeLists.txt，另一个是位于src目录下的，还有一个是位于test目录下的。

Message.hpp头文件包含以下内容:

.. code-block:: c++

  #pragma once
  #include <iosfwd>
  #include <string>
  class Message
  {
  public:
    Message(const std::string &m) : message_(m) {}
    friend std::ostream &operator<<(std::ostream &os, Message &obj)
    {
      return obj.printObject(os);
    }
  private:
    std::string message_;
    std::ostream &printObject(std::ostream &os);
  };
  std::string getUUID();

Message.cpp中有相应的实现：

.. code-block:: c++

  #include "Message.hpp"
  #include <iostream>
  #include <string>
  #ifdef HAVE_UUID
  #include <uuid/uuid.h>
  #endif
  std::ostream &Message::printObject(std::ostream &os)
  {
    os << "This is my very nice message: " << std::endl;
    os << message_ << std::endl;
    os << "...and here is its UUID: " << getUUID();
    return os;
  }
  #ifdef HAVE_UUID
  std::string getUUID()
  {
    uuid_t uuid;
    uuid_generate(uuid);
    char uuid_str[37];
    uuid_unparse_lower(uuid, uuid_str);
    uuid_clear(uuid);
    std::string uuid_cxx(uuid_str);
    return uuid_cxx;
  }
  #else
  std::string getUUID()
  {
    return "Ooooops, no UUID for you!";
  }
  #endif

最后，示例hello-world.cpp内容如下:

.. code-block:: c++

  #include <cstdlib>
  #include <iostream>
  #include "Message.hpp"
  int main()
  {
    Message say_hello("Hello, CMake World!");
    std::cout << say_hello << std::endl;
    Message say_goodbye("Goodbye, CMake World");
    std::cout << say_goodbye << std::endl;
    return EXIT_SUCCESS;
  }

**具体实施**

我们先来看一下主CMakeLists.txt:

1 声明CMake最低版本，并定义一个C++11项目。请注意，我们已经为我们的项目设置了一个版本，在project中使用VERSION进行指定:

.. code-block:: cmake

  # CMake 3.6 needed for IMPORTED_TARGET option
  # to pkg_search_module
  cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
  project(recipe-01
  LANGUAGES CXX
  VERSION 1.0.0
  )
  # <<< General set up >>>
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 用户可以通过CMAKE_INSTALL_PREFIX变量定义安装目录。CMake会给这个变量设置一个默认值：Windows上的C:\Program Files和Unix上的/usr/local。
我们将会打印安装目录的信息：

.. code-block:: cmake

  message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX}")

3 默认情况下，我们更喜欢以Release的方式配置项目。用户可以通过CMAKE_BUILD_TYPE设置此变量，从而改变配置类型，我们将检查是否存在这种情况。
如果没有，将设置为默认值:

.. code-block:: cmake

  if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")

4 接下来，告诉CMake在何处构建可执行、静态和动态库目标。便于在用户不打算安装项目的情况下，访问这些构建目标。
这里使用标准CMake的GNUInstallDirs.cmake模块。这将确保的项目布局的合理性和可移植性：

.. code-block:: cmake

  include(GNUInstallDirs)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
      ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
      ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
      ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

5 虽然，前面的命令配置了构建目录中输出的位置，但是需要下面的命令来配置可执行程序、库以及安装前缀中包含的文件的位置。
  它们大致遵循相同的布局，但是我们定义了新的INSTALL_LIBDIR、INSTALL_BINDIR、INSTALL_INCLUDEDIR和INSTALL_CMAKEDIR变量。当然，也可以覆盖这些变量：

.. code-block:: cmake

  # Offer the user the choice of overriding the installation directories
  set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
  set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
  set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
  if(WIN32 AND NOT CYGWIN)
      set(DEF_INSTALL_CMAKEDIR CMake)
  else()
      set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
  endif()
  set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

6 报告组件安装的路径:

.. code-block:: cmake

  # Report to user
  foreach(p LIB BIN INCLUDE CMAKE)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path )
    message(STATUS "Installing ${p} components to ${_path}")
    unset(_path)
  endforeach()

7 主CMakeLists.txt文件中的最后一个指令添加src子目录，启用测试，并添加tests子目录：

.. code-block:: cmake

  add_subdirectory(src)
  enable_testing()
  add_subdirectory(tests)

现在我们继续分析src/CMakeLists.txt，其定义了构建的实际目标：

1 我们的项目依赖于UUID库：

.. code-block:: cmake

  # Search for pkg-config and UUID
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_search_module(UUID uuid IMPORTED_TARGET)
    if(TARGET PkgConfig::UUID)
      message(STATUS "Found libuuid")
      set(UUID_FOUND TRUE)
    endif()
  endif()

2 我们希望建立一个动态库，将该目标声明为message-shared:

.. code-block:: cmake

  add_library(message-shared SHARED "")

3 这个目标由target_sources命令指定:

.. code-block:: cmake

  target_sources(message-shared
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Message.cpp
    )

4 我们为目标声明编译时定义和链接库。请注意，所有这些都是PUBLIC，以确保所有依赖的目标将正确继承它们:

.. code-block:: cmake

    target_compile_definitions(message-shared
    PUBLIC
        $<$<BOOL:${UUID_FOUND}>:HAVE_UUID>
    )
    target_link_libraries(message-shared
    PUBLIC
        $<$<BOOL:${UUID_FOUND}>:PkgConfig::UUID>
    )

5 然后设置目标的附加属性:

.. code-block:: cmake

  set_target_properties(message-shared
    PROPERTIES
      POSITION_INDEPENDENT_CODE 1
      SOVERSION ${PROJECT_VERSION_MAJOR}
      OUTPUT_NAME "message"
      DEBUG_POSTFIX "_d"
      PUBLIC_HEADER "Message.hpp"
      MACOSX_RPATH ON
      WINDOWS_EXPORT_ALL_SYMBOLS ON
    )

6 最后，为“Hello, world”程序添加可执行目标:

.. code-block:: bash

  add_executable(hello-world_wDSO hello-world.cpp)
  hello-world_wDSO可执行目标，会链接到动态库：

  target_link_libraries(hello-world_wDSO
    PUBLIC
        message-shared
    )

src/CMakeLists.txt文件中，还包含安装指令。考虑这些之前，我们需要设置可执行文件的RPATH：

1 使用CMake路径操作，我们可以设置message_RPATH变量。这将为GNU/Linux和macOS设置适当的RPATH:

.. code-block:: bash

  RPATH
  file(RELATIVE_PATH _rel ${CMAKE_INSTALL_PREFIX}/${INSTALL_BINDIR} ${CMAKE_INSTALL_PREFIX})
  if(APPLE)
      set(_rpath "@loader_path/${_rel}")
  else()
      set(_rpath "\$ORIGIN/${_rel}")
  endif()
  file(TO_NATIVE_PATH "${_rpath}/${INSTALL_LIBDIR}" message_RPATH)

2 现在，可以使用这个变量来设置可执行目标hello-world_wDSO的RPATH(通过目标属性实现)。我们也可以设置额外的属性，稍后会对此进行更多的讨论:

.. code-block:: cmake

  set_target_properties(hello-world_wDSO
    PROPERTIES
      MACOSX_RPATH ON
      SKIP_BUILD_RPATH OFF
      BUILD_WITH_INSTALL_RPATH OFF
      INSTALL_RPATH "${message_RPATH}"
      INSTALL_RPATH_USE_LINK_PATH ON
    )

3 终于可以安装库、头文件和可执行文件了！使用CMake提供的install命令来指定安装位置。注意，路径是相对的，我们将在后续进一步讨论这一点:

.. code-block:: cmake

  install(
    TARGETS
      message-shared
      hello-world_wDSO
    ARCHIVE
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib
    RUNTIME
      DESTINATION ${INSTALL_BINDIR}
      COMPONENT bin
    LIBRARY
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib
    PUBLIC_HEADER
      DESTINATION ${INSTALL_INCLUDEDIR}/message
      COMPONENT dev
    )

4 tests目录中的CMakeLists.txt文件包含简单的指令，以确保“Hello, World”可执行文件能够正确运行：

.. code-block:: cmake

  add_test(
    NAME test_shared
    COMMAND $<TARGET_FILE:hello-world_wDSO>
    )

6 现在让我们配置、构建和安装项目，并查看结果。添加安装指令时，CMake就会生成一个名为install的新目标，该目标将运行安装规则:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake -G"Unix Makefiles" -DCMAKE_INSTALL_PREFIX=$HOME/Software/recipe-01
  $ cmake --build . --target install

GNU/Linux构建目录的内容如下:

.. code-block:: bash

  build
  ├── bin
  │    └── hello-world_wDSO
  ├── CMakeCache.txt
  ├── CMakeFiles
  ├── cmake_install.cmake
  ├── CTestTestfile.cmake
  ├── install_manifest.txt
  ├── lib64
  │    ├── libmessage.so -> libmessage.so.1
  │    └── libmessage.so.1
  ├── Makefile
  ├── src
  ├── Testing
  └── tests

另一方面，在安装位置，可以找到如下的目录结构:

.. code-block:: bash

  $HOME/Software/recipe-01/
  ├── bin
  │    └── hello-world_wDSO
  ├── include
  │    └── message
  │        └── Message.hpp
  └── lib64
      ├── libmessage.so -> libmessage.so.1
      └── libmessage.so.1

这意味着安装指令中给出的位置，是相对于用户给定的CMAKE_INSTALL_PREFIX路径。


10.2 生成输出头文件
-------------------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-10/recipe-02 中找到，其中有一个C++示例。
  该示例在CMake 3.6版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

设想一下，当我们的小型库非常受欢迎时，许多人都在使用它。然而，一些客户希望在安装时使用静态库，而另一些客户也注意到所有符号在动态库中都是可见的。
最佳方式是规定动态库只公开最小的符号，从而限制代码中定义的对象和函数对外的可见性。我们希望在默认情况下，动态库定义的所有符号都对外隐藏。这将使得项目的贡献者，能够清楚地划分库和外部代码之间的接口，因为他们必须显式地标记所有要在项目外部使用的符号。因此，我们需要完成以下工作：

* 使用同一组源文件构建动态库和静态库
* 确保正确分隔动态库中符号的可见性

第1章第3节中，已经展示了CMake提供了与平台无关的方式实现的功能。但是，没有处理符号可见性的问题。我们将用当前的配方重新讨论这两点。

**准备工作**

我们仍将使用与前一个示例中基本相同的代码，但是我们需要修改src/CMakeLists.txt和Message.hpp头文件。后者将包括新的、自动生成的头文件messageExport.h:

.. code-block:: c++

  #pragma once
  #include
  #include
  #include "messageExport.h"
  class message_EXPORT Message
  {
  public:
    Message(const std::string &m) : message_(m) {}
    friend std::ostream &operator<<(std::ostream &os, Message &obj)
    {
      return obj.printObject(os);
    }
  private:
    std::string message_;
    std::ostream &printObject(std::ostream &os);
  };
  std::string getUUID();

Message类的声明中引入了message_EXPORT预处理器指令，这个指令将让编译器生成对库的用户可见的符号。

**具体实施**

除了项目的名称外，主CMakeLists.txt文件没有改变。首先，看看src子目录中的CMakeLists.txt文件，所有工作实际上都在这里进行。
我们将重点展示对之前示例的修改之处:


1 为消息传递库声明SHARED库目标及其源。注意，编译定义和链接库没有改变:

.. code-block:: cmake

  add_library(message-shared SHARED "")
  target_sources(message-shared
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Message.cpp
    )
  target_compile_definitions(message-shared
      PUBLIC
        $<$<BOOL:${UUID_FOUND}>:HAVE_UUID>
    )
  target_link_libraries(message-shared
    PUBLIC
        $<$<BOOL:${UUID_FOUND}>:PkgConfig::UUID>
    )

2 设置目标属性。将${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}/messageExport.h头文件添加到公共头列表中，作为PUBLIC_HEADER目标属性的参数。
CXX_VISIBILITY_PRESET置和VISIBILITY_INLINES_HIDDEN属性将在下一节中讨论:

.. code-block:: cmake

  set_target_properties(message-shared
    PROPERTIES
      POSITION_INDEPENDENT_CODE 1
      CXX_VISIBILITY_PRESET hidden
      VISIBILITY_INLINES_HIDDEN 1
      SOVERSION ${PROJECT_VERSION_MAJOR}
      OUTPUT_NAME "message"
      DEBUG_POSTFIX "_d"
      PUBLIC_HEADER "Message.hpp;${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}/messageExport.h"
      MACOSX_RPATH ON
    )

3 包含GenerateExportHeader.cmake模块并调用generate_export_header函数，这将在构建目录的子目录中生成messageExport.h头文件。
我们将稍后会详细讨论这个函数和生成的头文件:

.. code-block:: cmake

  include(GenerateExportHeader)
  generate_export_header(message-shared
    BASE_NAME "message"
    EXPORT_MACRO_NAME "message_EXPORT"
    EXPORT_FILE_NAME "${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}/messageExport.h"
    DEPRECATED_MACRO_NAME "message_DEPRECATED"
    NO_EXPORT_MACRO_NAME "message_NO_EXPORT"
    STATIC_DEFINE "message_STATIC_DEFINE"
    NO_DEPRECATED_MACRO_NAME "message_NO_DEPRECATED"
    DEFINE_NO_DEPRECATED
    )

4 当要更改符号的可见性(从其默认值-隐藏值)时，都应该包含导出头文件。我们已经在Message.hpp头文件例这样做了，因为想在库中公开一些符号。
现在将${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}目录作为message-shared目标的PUBLIC包含目录列出：

.. code-block:: cmake

  target_include_directories(message-shared
    PUBLIC
        ${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}
    )

现在，可以将注意力转向静态库的生成：

1 添加一个库目标来生成静态库。将编译与静态库相同的源文件，以获得此动态库目标：

.. code-block:: cmake

  add_library(message-static STATIC "")
  target_sources(message-static
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Message.cpp
    )

2 设置编译器定义，包含目录和链接库，就像我们为动态库目标所做的一样。但请注意，我们添加了message_STATIC_DEFINE编译时宏定义，
为了确保我们的符号可以适当地暴露:

.. code-block:: cmake

  target_compile_definitions(message-static
    PUBLIC
        message_STATIC_DEFINE
        $<$<BOOL:${UUID_FOUND}>:HAVE_UUID>
    )
  target_include_directories(message-static
        PUBLIC
        ${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}
    )
  target_link_libraries(message-static
    PUBLIC
        $<$<BOOL:${UUID_FOUND}>:PkgConfig::UUID>
    )

3 还设置了message-static目标的属性:

.. code-block:: cmake

  set_target_properties(message-static
    PROPERTIES
      POSITION_INDEPENDENT_CODE 1
      ARCHIVE_OUTPUT_NAME "message"
      DEBUG_POSTFIX "_sd"
      RELEASE_POSTFIX "_s"
      PUBLIC_HEADER "Message.hpp;${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}/messageExport.h"
    )

4 除了链接到消息动态库目标的hello-world_wDSO可执行目标之外，还定义了另一个可执行目标hello-world_wAR，这个链接指向静态库:

.. code-block:: cmake

  add_executable(hello-world_wAR hello-world.cpp)
  target_link_libraries(hello-world_wAR
      PUBLIC
          message-static
      )

5 安装指令现在多了message-static和hello-world_wAR目标，其他没有改变:

.. code-block:: cmake

  install(
    TARGETS
      message-shared
      message-static
      hello-world_wDSO
      hello-world_wAR
    ARCHIVE
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib
    RUNTIME
      DESTINATION ${INSTALL_BINDIR}
      COMPONENT bin
    LIBRARY
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib
    PUBLIC_HEADER
      DESTINATION ${INSTALL_INCLUDEDIR}/message
      COMPONENT dev
    )

**工作原理**

此示例演示了，如何设置动态库的符号可见性。最好的方式是在默认情况下隐藏所有符号，显式地只公开那些需要使用的符号。这需要分为两步实现。
首先，需要指示编译器隐藏符号。当然，不同的编译器将有不同的可用选项，并且直接在CMakeLists.txt中设置这些选项并不是是跨平台的。
CMake通过在动态库目标上设置两个属性，提供了一种健壮的跨平台方法来设置符号的可见性：

* CXX_VISIBILITY_PRESET hidden：这将隐藏所有符号，除非显式地标记了其他符号。当使用GNU编译器时，这将为目标添加-fvisibility=hidden标志。
* VISIBILITY_INLINES_HIDDEN 1：这将隐藏内联函数的符号。如果使用GNU编译器，这对应于-fvisibility-inlines-hidden

Windows上，这都是默认行为。实际上，我们需要在前面的示例中通过设置WINDOWS_EXPORT_ALL_SYMBOLS属性为ON来覆盖它。

如何标记可见的符号？这由预处理器决定，因此需要提供相应的预处理宏，这些宏可以扩展到所选平台上，以便编译器能够理解可见性属性。
CMake中有现成的GenerateExportHeader.cmake模块。这个模块定义了generate_export_header函数，我们调用它的过程如下：

.. code-block:: cmake

  include(GenerateExportHeader)
  generate_export_header(message-shared
    BASE_NAME "message"
    EXPORT_MACRO_NAME "message_EXPORT"
    EXPORT_FILE_NAME "${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}/messageExport.h"
    DEPRECATED_MACRO_NAME "message_DEPRECATED"
    NO_EXPORT_MACRO_NAME "message_NO_EXPORT"
    STATIC_DEFINE "message_STATIC_DEFINE"
    NO_DEPRECATED_MACRO_NAME "message_NO_DEPRECATED"
    DEFINE_NO_DEPRECATED
    )

该函数生成messageExport.h头文件，其中包含预处理器所需的宏。根据EXPORT_FILE_NAME选项的请求，在目录${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}中生成该文件。
如果该选项为空，则头文件将在当前二进制目录中生成。这个函数的第一个参数是现有的目标(示例中是message- shared)，函数的基本调用只需要传递现有目标的名称即可。
可选参数，用于细粒度的控制所有生成宏，也可以传递：

* BASE_NAME：设置生成的头文件和宏的名称。
* EXPORT_MACRO_NAME：设置导出宏的名称。
* EXPORT_FILE_NAME：设置导出头文件的名称。
* DEPRECATED_MACRO_NAME：设置弃用宏的名称。这是用来标记将要废弃的代码，如果客户使用该宏定义，编译器将发出一个将要废弃的警告。
* NO_EXPORT_MACRO_NAME：设置不导出宏的名字。
* TATIC_DEFINE：用于定义宏的名称，以便使用相同源编译静态库时使用。
* NO_DEPRECATED_MACRO_NAME：设置宏的名称，在编译时将“将要废弃”的代码排除在外。
* DEFINE_NO_DEPRECATED：指示CMake生成预处理器代码，以从编译中排除“将要废弃”的代码。

GNU/Linux上，使用GNU编译器，CMake将生成以下messageExport.h头文件:

.. code-block:: c++

  #ifndef message_EXPORT_H
  #define message_EXPORT_H
  #ifdef message_STATIC_DEFINE
  # define message_EXPORT
  # define message_NO_EXPORT
  #else
  # ifndef message_EXPORT
  # ifdef message_shared_EXPORTS
  /* We are building this library */
  # define message_EXPORT __attribute__((visibility("default")))
  # else
  /* We are using this library */
  # define message_EXPORT __attribute__((visibility("default")))
  # endif
  # endif
  # ifndef message_NO_EXPORT
  # define message_NO_EXPORT __attribute__((visibility("hidden")))
  # endif
  #endif
  #ifndef message_DEPRECATED
  # define message_DEPRECATED __attribute__ ((__deprecated__))
  #endif
  #ifndef message_DEPRECATED_EXPORT
  # define message_DEPRECATED_EXPORT message_EXPORT message_DEPRECATED
  #endif
  #ifndef message_DEPRECATED_NO_EXPORT
  # define message_DEPRECATED_NO_EXPORT message_NO_EXPORT message_DEPRECATED
  #endif
  #if 1 /* DEFINE_NO_DEPRECATED */
  # ifndef message_NO_DEPRECATED
  # define message_NO_DEPRECATED
  # endif
  #endif
  #endif

我们可以使用message_EXPORT宏，预先处理用户公开类和函数。弃用可以通过在前面加上message_DEPRECATED宏来实现。

从messageExport.h头文件的内容可以看出，所有符号都应该在静态库中可见，这就是message_STATIC_DEFINE宏起了作用。当声明了目标，我们就将其设置为编译时定义。静态库的其他目标属性如下:

* ARCHIVE_OUTPUT_NAME "message"：这将确保库文件的名称是message，而不是message-static。
* DEBUG_POSTFIX "_sd"：这将把给定的后缀附加到库名称中。当目标构建类型为Release时，为静态库添加”_sd”后缀。
* RELEASE_POSTFIX "_s"：这与前面的属性类似，当目标构建类型为Release时，为静态库添加后缀“_s”。


10.3 输出目标
-----------------------------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-10/recipe-03 中找到，其中有一个C++示例。
  该示例在CMake 3.6版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

可以假设，消息库在开源社区取得了巨大的成功。人们非常喜欢它，并在自己的项目中使用它将消息打印到屏幕上。用户特别喜欢每个打印的消息都有惟一的标识符。
但用户也希望，当他们编译并安装了库，库就能更容易找到。这个示例将展示CMake如何让我们导出目标，以便其他使用CMake的项目可以轻松地获取它们。

**准备工作**

源代码与之前的示例一致，项目结构如下:

.. code-block:: bash

  .
  ├── cmake
  │    └── messageConfig.cmake.in
  ├── CMakeLists.txt
  ├── src
  │    ├── CMakeLists.txt
  │    ├── hello- world.cpp
  │    ├── Message.cpp
  │    └── Message.hpp
  └── tests
      ├── CMakeLists.txt
      └── use_target
          ├── CMakeLists.txt
          └── use_message.cpp

注意，cmake子目录中添加了一个messageConfig.cmake.in。这个文件将包含导出的目标，还添加了一个测试来检查项目的安装和导出是否按预期工作。

**具体实施**

同样，主CMakeLists.txt文件相对于前一个示例来说没有变化。移动到包含我们的源代码的子目录src中：

1 需要找到UUID库，可以重用之前示例中的代码：

.. code-block:: cmake

  # Search for pkg-config and UUID
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
      pkg_search_module(UUID uuid IMPORTED_TARGET)
      if(TARGET PkgConfig::UUID)
          message(STATUS "Found libuuid")
          set(UUID_FOUND TRUE)
      endif()
  endif()

2 接下来，设置动态库目标并生成导出头文件：

.. code-block:: cmake

  add_library(message-shared SHARED "")
  include(GenerateExportHeader)
  generate_export_header(message-shared
    BASE_NAME "message"
    EXPORT_MACRO_NAME "message_EXPORT"
    EXPORT_FILE_NAME "${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}/messageExport.h"
    DEPRECATED_MACRO_NAME "message_DEPRECATED"
    NO_EXPORT_MACRO_NAME "message_NO_EXPORT"
    STATIC_DEFINE "message_STATIC_DEFINE"
    NO_DEPRECATED_MACRO_NAME "message_NO_DEPRECATED"
    DEFINE_NO_DEPRECATED
    )
  target_sources(message-shared
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Message.cpp
    )

3 为目标设置了PUBLIC和INTERFACE编译定义。注意$<INSTALL_INTERFACE:...>生成器表达式的使用：

.. code-block:: cmake

  target_compile_definitions(message-shared
  PUBLIC
      $<$<BOOL:${UUID_FOUND}>:HAVE_UUID>
  INTERFACE
      $<INSTALL_INTERFACE:USING_message>
  )

4 链接库和目标属性与前一个示例一样：

.. code-block:: cmake

  target_link_libraries(message-static
    PUBLIC
        $<$<BOOL:${UUID_FOUND}>:PkgConfig::UUID>
    )
  set_target_properties(message-static
      PROPERTIES
      POSITION_INDEPENDENT_CODE 1
      ARCHIVE_OUTPUT_NAME "message"
      DEBUG_POSTFIX "_sd"
      RELEASE_POSTFIX "_s"
      PUBLIC_HEADER "Message.hpp;${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}/messageExport.h"
    )

5 可执行文件的生成，与前一个示例中使用的命令完全相同：

.. code-block:: cmake

  add_executable(hello-world_wDSO hello-world.cpp)
  target_link_libraries(hello-world_wDSO
    PUBLIC
        message-shared
    )
  # Prepare RPATH
  file(RELATIVE_PATH _rel ${CMAKE_INSTALL_PREFIX}/${INSTALL_BINDIR} ${CMAKE_INSTALL_PREFIX})
  if(APPLE)
      set(_rpath "@loader_path/${_rel}")
  else()
      set(_rpath "\$ORIGIN/${_rel}")
  endif()
  file(TO_NATIVE_PATH "${_rpath}/${INSTALL_LIBDIR}" message_RPATH)
  set_target_properties(hello-world_wDSO
    PROPERTIES
      MACOSX_RPATH ON
      SKIP_BUILD_RPATH OFF
      BUILD_WITH_INSTALL_RPATH OFF
      INSTALL_RPATH "${message_RPATH}"
      INSTALL_RPATH_USE_LINK_PATH ON
    )
  add_executable(hello-world_wAR hello-world.cpp)
  target_link_libraries(hello-world_wAR
    PUBLIC
        message-static
    )

现在，来看看安装规则：

1 因为CMake可以正确地将每个目标放在正确的地方，所以把目标的安装规则都列在一起。这次，添加了EXPORT关键字，这样CMake将为目标生成一个导出的目标文件：

.. code-block:: cmake

  install(
    TARGETS
      message-shared
      message-static
      hello-world_wDSO
      hello-world_wAR
    EXPORT
        messageTargets
    ARCHIVE
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib
    RUNTIME
      DESTINATION ${INSTALL_BINDIR}
      COMPONENT bin
    LIBRARY
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib
    PUBLIC_HEADER
      DESTINATION ${INSTALL_INCLUDEDIR}/message
      COMPONENT dev
    )

2 自动生成的导出目标文件称为messageTargets.cmake，需要显式地指定它的安装规则。这个文件的目标是INSTALL_CMAKEDIR，在主CMakeLists.txt文件中定义:

.. code-block:: cmake

  install(
    EXPORT
        messageTargets
    NAMESPACE
        "message::"
    DESTINATION
        ${INSTALL_CMAKEDIR}
    COMPONENT
        dev
    )

3 最后，需要生成正确的CMake配置文件。这些将确保下游项目能够找到消息库导出的目标。为此，首先包括CMakePackageConfigHelpers.cmake标准模块：

.. code-block:: bash

  include(CMakePackageConfigHelpers)
  让CMake为我们的库，生成一个包含版本信息的文件:

  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/messageConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

4 使用configure_package_config_file函数，我们生成了实际的CMake配置文件。这是基于模板cmake/messageConfig.cmake.in文件:

.. code-block:: cmake

  configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/messageConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/messageConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CMAKEDIR}
    )

5 最后，为这两个自动生成的配置文件设置了安装规则:

.. code-block:: cmake

  install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/messageConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/messageConfigVersion.cmake
    DESTINATION
        ${INSTALL_CMAKEDIR}
    )

cmake/messageConfig.cmake的内容是什么？该文件的顶部有相关的说明，可以作为用户文档供使用者查看。让我们看看实际的CMake命令:

1 占位符将使用configure_package_config_file命令进行替换:

.. code-block:: bash

  @PACKAGE_INIT@

2 包括为目标自动生成的导出文件:

.. code-block:: cmake

  include("${CMAKE_CURRENT_LIST_DIR}/messageTargets.cmake")

3 检查静态库和动态库，以及两个“Hello, World”可执行文件是否带有CMake提供的check_required_components函数：

.. code-block:: cmake

  check_required_components(
      "message-shared"
      "message-static"
      "message-hello-world_wDSO"
      "message-hello-world_wAR"
    )

4 检查目标PkgConfig::UUID是否存在。如果没有，我们再次搜索UUID库(只在非Windows操作系统下有效):

.. code-block:: cmake

  if(NOT WIN32)
    if(NOT TARGET PkgConfig::UUID)
      find_package(PkgConfig REQUIRED QUIET)
      pkg_search_module(UUID REQUIRED uuid IMPORTED_TARGET)
    endif()
  endif()

测试一下：

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake -DCMAKE_INSTALL_PREFIX=$HOME/Software/recipe-03 ..
  $ cmake --build . --target install

安装树应该如下所示：

.. code-block:: bash

  $HOME/Software/recipe-03/
  ├── bin
  │    ├── hello-world_wAR
  │    └── hello-world_wDSO
  ├── include
  │    └── message
  │        ├── messageExport.h
  │        └── Message.hpp
  ├── lib64
  │    ├── libmessage_s.a
  │    ├── libmessage.so -> libmessage.so.1
  │    └── libmessage.so.1
  └── share
      └── cmake
          └── recipe-03
              ├── messageConfig.cmake
              ├── messageConfigVersion.cmake
              ├── messageTargets.cmake
              └── messageTargets-release.cmake

出现了一个share子目录，其中包含我们要求CMake自动生成的所有文件。现在开始，消息库的用户可以在他们自己的CMakeLists.txt文件中找到消息库，
只要他们设置message_DIR的CMake变量，指向安装树中的share/cmake/message目录:

.. code-block:: cmake

  find_package(message 1 CONFIG REQUIRED)

**工作原理**

这个示例涵盖了很多领域。对于构建系统将要执行的操作，CMake目标是一个非常有用的抽象概念。
使用PRIVATE、PUBLIC和INTERFACE关键字，我们可以设置项目中的目标进行交互。在实践中，这允许我们定义目标A的依赖关系，将如何影响目标B(依赖于A)。
如果库维护人员提供了适当的CMake配置文件，那么只需很少的CMake命令就可以轻松地解决所有依赖关系。

这个问题可以通过遵循message-static、message-shared、hello-world_wDSO和hello-world_wAR目标概述的模式来解决。
我们将单独分析message-shared目标的CMake命令，这里只是进行一般性讨论：

1 生成目标在项目构建中列出其依赖项。对UUID库的链接是 message-shared的PUBLIC需求，因为它将用于在项目中构建目标和在下游项目中构建目标。
编译时宏定义和包含目录需要在PUBLIC级或INTERFACE级目标上进行设置。它们实际上是在项目中构建目标时所需要的，其他的只与下游项目相关。此外，
其中一些只有在项目安装之后才会相关联。这里使用了$<BUILD_INTERFACE:...>和$<INSTALL_INTERFACE:...>生成器表达式。只有消息库外部的下游目标才需要这些，
也就是说，只有在安装了目标之后，它们才会变得可见。我们的例子中，应用如下:

* 只有在项目中使用了message-shared库，那么$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}>才会扩
  展成${CMAKE_BINARY_DIR}/${INSTALL_INCLUDEDIR}

* 只有在message-shared库在另一个构建树中，作为一个已导出目标，那么$<INSTALL_INTERFACE:${INSTALL_INCLUDEDIR}>将会扩展成${INSTALL_INCLUDEDIR}

2 描述目标的安装规则，包括生成文件的名称。

3 描述CMake生成的导出文件的安装规则messageTargets.cmake文件将安装到INSTALL_CMAKEDIR。目标导出文件的安装规则的名称空间选项，
将把给定字符串前置到目标的名称中，这有助于避免来自不同项目的目标之间的名称冲突。INSTALL_CMAKEDIR变量是在主CMakeLists.txt文件中设置的:

.. code-block:: cmake

  if(WIN32 AND NOT CYGWIN)
      set(DEF_INSTALL_CMAKEDIR CMake)
  else()
      set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
  endif()
  set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

CMakeLists.txt的最后一部分生成配置文件。包括CMakePackageConfigHelpers.cmake模块，分三步完成:

1 调用write_basic_package_version_file函数生成一个版本文件包。宏的第一个参数是版本控制文件的路径：messageConfigVersion.cmake。
版本格式为Major.Minor.Patch，并使用PROJECT_VERSION指定版本，还可以指定与库的新版本的兼容性。例子中，当库具有相同的主版本时，为了保证兼容性，
使用了相同的SameMajorVersion参数。

2 接下来，配置模板文件messageConfig.cmake.in，该文件位于cmake子目录中。

3 最后，为新生成的文件设置安装规则。两者都将安装在INSTALL_CMAKEDIR下。


10.4 安装超级构建
----------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-10/recipe-04 中找到，其中有一个C++示例。
  该示例在CMake 3.6版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

我们的消息库取得了巨大的成功，许多其他程序员都使用它，并且非常满意。也希望在自己的项目中使用它，但是不确定如何正确地管理依赖关系。
可以用自己的代码附带消息库的源代码，但是如果该库已经安装在系统上了应该怎么做呢？第8章，展示了超级构建的场景，但是不确定如何安装这样的项目。
本示例将带您了解安装超级构建的安装细节。

**准备工作**

此示例将针对消息库，构建一个简单的可执行链接。项目布局如下:

.. code-block:: bash

  ├── cmake
  │    ├── install_hook.cmake.in
  │    └── print_rpath.py
  ├── CMakeLists.txt
  ├── external
  │    └── upstream
  │        ├── CMakeLists.txt
  │        └── message
  │            └── CMakeLists.txt
  └── src
      ├── CMakeLists.txt
      └── use_message.cpp

主CMakeLists.txt文件配合超级构建，external子目录包含处理依赖项的CMake指令。cmake子目录包含一个Python脚本和一个模板CMake脚本。
这些将用于安装方面的微调，CMake脚本首先进行配置，然后调用Python脚本打印use_message可执行文件的RPATH:

.. code-block:: python

  import shlex
  import subprocess
  import sys
  def main():
    patcher = sys.argv[1]
    elfobj = sys.argv[2]
    tools = {'patchelf': '--print-rpath', 'chrpath': '--list', 'otool': '-L'}
    if patcher not in tools.keys():
    raise RuntimeError('Unknown tool {}'.format(patcher))
    cmd = shlex.split('{:s} {:s} {:s}'.format(patcher, tools[patcher], elfobj))
    rpath = subprocess.run(
        cmd,
        bufsize=1,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True)
    print(rpath.stdout)
  if __name__ == "__main__":
    main()

使用平台原生工具可以轻松地打印RPATH，稍后我们将在本示例中讨论这些工具。

最后，src子目录包含项目的CMakeLists.txt和源文件。use_message.cpp源文件包含以下内容:

.. code-block:: c++

  #include <cstdlib>
  #include <iostream>
  #ifdef USING_message
  #include <message/Message.hpp>
  void messaging()
  {
    Message say_hello("Hello, World! From a client of yours!");
    std::cout << say_hello << std::endl;
    Message say_goodbye("Goodbye, World! From a client of yours!");
    std::cout << say_goodbye << std::endl;
  }
  #else
  void messaging()
  {
    std::cout << "Hello, World! From a client of yours!" << std::endl;
    std::cout << "Goodbye, World! From a client of yours!" << std::endl;
  }
  #endif
  int main()
  {
    messaging();
    return EXIT_SUCCESS;
  }

**具体实施**

我们将从主CMakeLists.txt文件开始，它用来协调超级构建:

1 与之前的示例相同。首先声明一个C++11项目，设置了默认安装路径、构建类型、目标的输出目录，以及安装树中组件的布局:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
  project(recipe-04
    LANGUAGES CXX
    VERSION 1.0.0
    )
  # <<< General set up >>>
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")
  message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX}")
  include(GNUInstallDirs)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
  # Offer the user the choice of overriding the installation directories
  set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
  set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
  set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
  if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKEDIR CMake)
  else()
    set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
  endif()
  set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")
  # Report to user
  foreach(p LIB BIN INCLUDE CMAKE)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path )
    message(STATUS "Installing ${p} components to ${_path}")
    unset(_path)
  endforeach()

2 设置了EP_BASE目录属性，这将为超构建中的子项目设置布局。所有子项目都将在CMAKE_BINARY_DIR的子项目文件夹下生成:

.. code-block:: cmake

  set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)

3 然后，声明STAGED_INSTALL_PREFIX变量。这个变量指向构建目录下的stage子目录，项目将在构建期间安装在这里。这是一种沙箱安装过程，让我们有机会检查整个超级构建的布局:

.. code-block:: cmake

  set(STAGED_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/stage)
  message(STATUS "${PROJECT_NAME} staged install: ${STAGED_INSTALL_PREFIX}")

4 添加external/upstream子目录。其中包括使用CMake指令来管理我们的上游依赖关系，在我们的例子中，就是消息库:

.. code-block:: cmake

  add_subdirectory(external/upstream)

5 然后，包含ExternalProject.cmake标准模块:

.. code-block:: cmake

  include(ExternalProject)

6 将自己的项目作为外部项目添加，调用ExternalProject_Add命令。SOURCE_DIR用于指定源位于src子目录中。我们会选择适当的CMake参数来配置我们的项目。
这里，使用STAGED_INSTALL_PREFIX作为子项目的安装目录:

.. code-block:: cmake

  ExternalProject_Add(${PROJECT_NAME}_core
    DEPENDS
      message_external
    SOURCE_DIR
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
      -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
      -DCMAKE_CXX_EXTENSIONS=${CMAKE_CXX_EXTENSIONS}
      -DCMAKE_CXX_STANDARD_REQUIRED=${CMAKE_CXX_STANDARD_REQUIRED}
      -Dmessage_DIR=${message_DIR}
    CMAKE_CACHE_ARGS
      -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
    BUILD_ALWAYS
      1
    )

7 现在，为use_message添加一个测试，并由recipe-04_core构建。这将运行use_message可执行文件的安装，即位于构建树中的安装:

.. code-block:: cmake

  enable_testing()
  add_test(
    NAME
        check_use_message
    COMMAND
        ${STAGED_INSTALL_PREFIX}/${INSTALL_BINDIR}/use_message
    )

8 最后，可以声明安装规则。因为所需要的东西都已经安装在暂存区域中，我们只要将暂存区域的内容复制到安装目录即可:

.. code-block:: cmake

  install(
    DIRECTORY
        ${STAGED_INSTALL_PREFIX}/
    DESTINATION
        .
    USE_SOURCE_PERMISSIONS
    )

9 使用SCRIPT参数声明一个附加的安装规则。CMake脚本的install_hook.cmake将被执行，但只在GNU/Linux和macOS上执行。
这个脚本将打印已安装的可执行文件的RPATH，并运行它。我们将在下一节详细地讨论这个问题：

.. code-block:: cmake

  if(UNIX)
    set(PRINT_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/cmake/print_rpath.py")
    configure_file(cmake/install_hook.cmake.in install_hook.cmake @ONLY)
    install(
      SCRIPT
        ${CMAKE_CURRENT_BINARY_DIR}/install_hook.cmake
      )
  endif()

-Dmessage_DIR=${message_DIR}已作为CMake参数传递给项目，这将正确设置消息库依赖项的位置。
message_DIR的值在external/upstream/message目录下的CMakeLists.txt文件中定义。这个文件处理依赖于消息库，让我们看看是如何处理的:

1 首先，搜索并找到包。用户可能已经在系统的某个地方安装了，并在配置时传递了message_DIR:

.. code-block:: cmake

  find_package(message 1 CONFIG QUIET)

2 如果找到了消息库，我们将向用户报告目标的位置和版本，并添加一个虚拟的message_external目标。这里，需要虚拟目标来正确处理超构建的依赖关系:

.. code-block:: cmake

  if(message_FOUND)
    get_property(_loc TARGET message::message-shared PROPERTY LOCATION)
    message(STATUS "Found message: ${_loc} (found version ${message_VERSION})")
    add_library(message_external INTERFACE) # dummy

3 如果没有找到这个库，我们将把它添加为一个外部项目，从在线Git存储库下载它，然后编译它。安装路径、构建类型和安装目录布局都是由主CMakeLists.txt文件设置，
C++编译器和标志也是如此。项目将安装到STAGED_INSTALL_PREFIX下，然后进行测试:

.. code-block:: cmake

  else()
    include(ExternalProject)
    message(STATUS "Suitable message could not be located, Building message instead.")
    ExternalProject_Add(message_external
      GIT_REPOSITORY
        https://github.com/dev-cafe/message.git
      GIT_TAG
        master
      UPDATE_COMMAND
        ""
      CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      CMAKE_CACHE_ARGS
        -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
      TEST_AFTER_INSTALL
        1
      DOWNLOAD_NO_PROGRESS
        1
      LOG_CONFIGURE
        1
      LOG_BUILD
        1
      LOG_INSTALL
        1
    )

4 最后，将message_DIR目录进行设置，为指向新构建的messageConfig.cmake文件指明安装路径。注意，这些路径被保存到CMakeCache中:

.. code-block:: cmake

    if(WIN32 AND NOT CYGWIN)
      set(DEF_message_DIR ${STAGED_INSTALL_PREFIX}/CMake)
    else()
      set(DEF_message_DIR ${STAGED_INSTALL_PREFIX}/share/cmake/message)
    endif()
    file(TO_NATIVE_PATH "${DEF_message_DIR}" DEF_message_DIR)
    set(message_DIR ${DEF_message_DIR}
      CACHE PATH "Path to internally built messageConfig.cmake" FORCE)
    endif()

我们终于准备好编译我们自己的项目，并成功地将其链接到消息库(无论是系统上已有的消息库，还是新构建的消息库)。由于这是一个超级构建，
src子目录下的代码是一个完全独立的CMake项目:

1 声明一个C++11项目：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
  project(recipe-04_core
    LANGUAGES CXX
    )
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  include(GNUInstallDirs)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

2 尝试找到消息库。超级构建中，正确设置message_DIR:

.. code-block:: cmake

  find_package(message 1 CONFIG REQUIRED)
  get_property(_loc TARGET message::message-shared PROPERTY LOCATION)
  message(STATUS "Found message: ${_loc} (found version ${message_VERSION})")

3 添加可执行目标use_message，该目标由use_message.cpp源文件创建，并连接到message::message-shared目标:

.. code-block:: cmake

  add_executable(use_message use_message.cpp)
  target_link_libraries(use_message
    PUBLIC
        message::message-shared
    )

4 为use_message设置目标属性。再次对RPATH进行设置:

.. code-block:: cmake

  # Prepare RPATH
  file(RELATIVE_PATH _rel ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR} ${CMAKE_INSTALL_PREFIX})
  if(APPLE)
    set(_rpath "@loader_path/${_rel}")
  else()
    set(_rpath "\$ORIGIN/${_rel}")
  endif()
  file(TO_NATIVE_PATH "${_rpath}/${CMAKE_INSTALL_LIBDIR}" use_message_RPATH)
  set_target_properties(use_message
    PROPERTIES
      MACOSX_RPATH ON
      SKIP_BUILD_RPATH OFF
      BUILD_WITH_INSTALL_RPATH OFF
      INSTALL_RPATH "${use_message_RPATH}"
      INSTALL_RPATH_USE_LINK_PATH ON
    )

5 最后，为use_message目标设置了安装规则:

.. code-block:: cmake

  install(
    TARGETS
        use_message
    RUNTIME
      DESTINATION ${CMAKE_INSTALL_BINDIR}
      COMPONENT bin
    )

现在瞧瞧CMake脚本模板install_hook.cmake.in的内容：

1 CMake脚本在我们的主项目范围之外执行，因此没有定义变量或目标的概念。因此，需要设置变量来保存已安装的use_message可执行文件的完整路径。
注意使用@INSTALL_BINDIR@，它将由configure_file解析：

.. code-block:: cmake

  set(_executable ${CMAKE_INSTALL_PREFIX}/@INSTALL_BINDIR@/use_message)

2 需要找到平台本机可执行工具，使用该工具打印已安装的可执行文件的RPATH。我们将搜索chrpath、patchelf和otool。当找到已安装的程序时，
向用户提供有用的状态信息，并且退出搜索：

.. code-block:: cmake

  set(_patcher)
  list(APPEND _patchers chrpath patchelf otool)
  foreach(p IN LISTS _patchers)
    find_program(${p}_FOUND
      NAMES
        ${p}
      )
    if(${p}_FOUND)
      set(_patcher ${p})
      message(STATUS "ELF patching tool ${_patcher} FOUND")
      break()
    endif()
  endforeach()

3 检查_patcher变量是否为空，这意味着PatchELF工具是否可用。当为空时，我们要进行的操作将会失败，所以会发出一个致命错误，提醒用户需要安装PatchELF工具:

.. code-block:: cmake

  if(NOT _patcher)
      message(FATAL_ERROR "ELF patching tool NOT FOUND!\nPlease install one of chrpath, patchelf or otool")

4 当PatchELF工具找到了，则继续。我们调用Python脚本print_rpath.py，将_executable变量作为参数传递给execute_process：

.. code-block:: cmake

    find_package(PythonInterp REQUIRED QUIET)
    execute_process(
      COMMAND
        ${PYTHON_EXECUTABLE} @PRINT_SCRIPT@ "${_patcher}"
      "${_executable}"
      RESULT_VARIABLE _res
      OUTPUT_VARIABLE _out
      ERROR_VARIABLE _err
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )

5 检查_res变量的返回代码。如果执行成功，将打印_out变量中捕获的标准输出流。否则，打印退出前捕获的标准输出和错误流:

.. code-block:: cmake

    if(_res EQUAL 0)
      message(STATUS "RPATH for ${_executable} is ${_out}")
    else()
      message(STATUS "Something went wrong!")
      message(STATUS "Standard output from print_rpath.py: ${_out}")
      message(STATUS "Standard error from print_rpath.py: ${_err}")
      message(FATAL_ERROR "${_patcher} could NOT obtain RPATH for ${_executable}")
    endif()
    endif()

6 再使用execute_process来运行已安装的use_message可执行目标:

.. code-block:: cmake

  execute_process(
    COMMAND ${_executable}
    RESULT_VARIABLE _res
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

7 最后，向用户报告execute_process的结果:

.. code-block:: cmake

  if(_res EQUAL 0)
    message(STATUS "Running ${_executable}:\n ${_out}")
  else()
    message(STATUS "Something went wrong!")
    message(STATUS "Standard output from running ${_executable}:\n ${_out}")
    message(STATUS "Standard error from running ${_executable}:\n ${_err}")
    message(FATAL_ERROR "Something went wrong with ${_executable}")
  endif()

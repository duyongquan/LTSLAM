.. highlight:: c++

.. default-domain:: cpp

==========================
第8章 超级构建模式
==========================

8.1 使用超级构建模式
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-8/recipe-01 中找到，其中有一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

本示例通过一个简单示例，介绍超级构建模式。我们将展示如何使用ExternalProject_Add命令来构建一个的“Hello, World”程序。


**具体实施**

让我们看一下根目录下的CMakeLists.txt：

1 声明一个C++11项目，以及CMake最低版本:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 为当前目录和底层目录设置EP_BASE目录属性:

.. code-block:: cmake

  set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)

3 包括ExternalProject.cmake标准模块。该模块提供了ExternalProject_Add函数:

.. code-block:: cmake

  include(ExternalProject)

4 “Hello, World”源代码通过调用ExternalProject_Add函数作为外部项目添加的。外部项目的名称为recipe-01_core:

.. code-block:: cmake

  ExternalProject_Add(${PROJECT_NAME}_core

5 使用SOURCE_DIR选项为外部项目设置源目录:

.. code-block:: bash

  SOURCE_DIR
  ${CMAKE_CURRENT_LIST_DIR}/src

6 src子目录包含一个完整的CMake项目。为了配置和构建它，通过CMAKE_ARGS选项将适当的CMake选项传递给外部项目。例子中，只需要通过C++编译器和C++标准的要求即可:

.. code-block:: bash

  CMAKE_ARGS
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DCMAKE_CXX_EXTENSIONS=${CMAKE_CXX_EXTENSIONS}
    -DCMAKE_CXX_STANDARD_REQUIRED=${CMAKE_CXX_STANDARD_REQUIRED}
    
7 我们还设置了C++编译器标志。这些通过使用CMAKE_CACHE_ARGS选项传递到ExternalProject_Add中:

  CMAKE_CACHE_ARGS
      -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}

8 我们配置外部项目，使它进行构建:

.. code-block:: bash

  BUILD_ALWAYS
      1

9 安装步骤不会执行任何操作(我们将在第4节中重新讨论安装，在第10章中安装超级构建，并编写安装程序):

.. code-block:: bash

  INSTALL_COMMAND
      ""
  )

现在，我们来看看src/CMakeLists.txt。由于我们将“Hello, World”源文件作为一个外部项目添加，这是一个独立项目的CMakeLists.txt文件:

1 这里声明CMake版本最低要求:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

2 声明一个C++项目：

.. code-block:: cmake

  project(recipe-01_core LANGUAGES CXX)

3 最终，使用hello-world.cpp源码文件生成可执行目标hello-world：

.. code-block:: cmake

  add_executable(hello-world hello-world.cpp)

4 配置构建项目：

.. code-block:: bash

  $ mkdir -p build
  $ cmake ..
  $ cmake --build .

5 构建目录的结构稍微复杂一些，subprojects文件夹的内容如下:

.. code-block:: bash

  build/subprojects/
  ├── Build
  │    └── recipe-01_core
  │        ├── CMakeCache.txt
  │        ├── CMakeFiles
  │        ├── cmake_install.cmake
  │        ├── hello-world
  │        └── Makefile
  ├── Download
  │    └── recipe-01_core
  ├── Install
  │    └── recipe-01_core
  ├── Stamp
  │    └── recipe-01_core
  │        ├── recipe-01_core-configure
  │        ├── recipe-01_core-done
  │        ├── recipe-01_core-download
  │        ├── recipe-01_core-install
  │        ├── recipe-01_core-mkdir
  │        ├── recipe-01_core-patch
  │        └── recipe-01_core-update
  └── tmp
      └── recipe-01_core
          ├── recipe-01_core-cache-.cmake
          ├── recipe-01_core-cfgcmd.txt
          └── recipe-01_core-cfgcmd.txt.in

recipe-01_core已经构建到build/subprojects子目录中，称为Build/recipe-01_core(这是我们设置的EP_BASE)。

hello-world可执行文件在Build/recipe-01_core下创建，其他子文件夹tmp/recipe-01_core和Stamp/recipe-01_core包含临时文件，比如：CMake缓存脚本recipe-01_core-cache-.cmake和已执行的外部构建项目的各步骤的时间戳文件。


8.1 使用超级构建模式
--------------------------


8.1 使用超级构建模式
--------------------------


8.1 使用超级构建模式
--------------------------

8.5 使用超级构建支持项目
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-8/recipe-05 中找到，其中有一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

ExternalProject和FetchContent是CMake库中两个非常强大的工具。经过前面的示例，我们应该相信超级构建方法，在管理复杂依赖关系的项目时是多么有用。
目前为止，我们已经展示了如何使用ExternalProject来处理以下问题:

* 存储在源树中的源
* 从在线服务器上，检索/获取可用的存档资源

前面的示例展示了，如何使用FetchContent处理开源Git存储库中可用的依赖项。本示例将展示，如何使用ExternalProject达到同样的效果。
最后，将介绍一个示例，该示例将在第10章第4节中重用。

**具体实施**

目前为止，建立超级构建的过程应该已经很熟悉了。让我们再次看看必要的步骤，从根目录的CMakeLists.txt开始:

1 声明一个C++11项目，并对项目构建类型的默认值进行设置。

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
  project(recipe-05 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")

2 设置EP_BASE目录属性。这将固定ExternalProject管理所有子项目的布局:

.. code-block:: cmake

  set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)

3 我们设置了STAGED_INSTALL_PREFIX。与之前一样，这个位置将作为依赖项的构建树中的安装目录:

.. code-block:: cmake

  set(STAGED_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/stage)
  message(STATUS "${PROJECT_NAME} staged install: ${STAGED_INSTALL_PREFIX}")

4 将external/upstream作为子目录添加：

.. code-block:: cmake

  add_subdirectory(external/upstream)

5 添加ExternalProject_Add，这样我们的项目也将由超级构建管理:

.. code-block:: cmake

  include(ExternalProject)
  ExternalProject_Add(${PROJECT_NAME}_core
    DEPENDS
        message_external
    SOURCE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
      -DCMAKE_CXX_EXTENSIONS=${CMAKE_CXX_EXTENSIONS}
      -DCMAKE_CXX_STANDARD_REQUIRED=${CMAKE_CXX_STANDARD_REQUIRED}
      -Dmessage_DIR=${message_DIR}
      CMAKE_CACHE_ARGS
      -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
      -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
    BUILD_ALWAYS
        1
    INSTALL_COMMAND
        ""
    )

6 external/upstream的CMakeLists.txt中只包含一条命令:

.. code-block:: cmake

  add_subdirectory(message)

跳转到message文件夹，我们会看到对消息库的依赖的常用命令:

1 首先，调用find_package找到一个合适版本的库:

.. code-block:: cmake

  find_package(message 1 CONFIG QUIET)

2 如果找到，会通知用户，并添加一个虚拟INTERFACE库:

.. code-block:: cmake

  get_property(_loc TARGET message::message-shared PROPERTY LOCATION)
  message(STATUS "Found message: ${_loc} (found version ${message_VERSION})")
  add_library(message_external INTERFACE) # dummy

3 如果没有找到，再次通知用户并继续使用ExternalProject_Add:

.. code-block:: cmake

  message(STATUS "Suitable message could not be located, Building message instead.")

4 该项目托管在一个公共Git库中，使用GIT_TAG选项指定下载哪个分支。和之前一样，将UPDATE_COMMAND选项置为空:

.. code-block:: cmake

  include(ExternalProject)
  ExternalProject_Add(message_external
    GIT_REPOSITORY
        https://github.com/dev-cafe/message.git
    GIT_TAG
        master
    UPDATE_COMMAND
        ""

5 外部项目使用CMake配置和构建，传递必要的构建选项:

.. code-block:: bash

    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
      -DCMAKE_CXX_EXTENSIONS=${CMAKE_CXX_EXTENSIONS}
      -DCMAKE_CXX_STANDARD_REQUIRED=${CMAKE_CXX_STANDARD_REQUIRED}
    CMAKE_CACHE_ARGS
        -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}

6 项目安装后进行测试:

.. code-block:: bash

    TEST_AFTER_INSTALL
        1
        
7 我们不希望看到下载进度，也不希望在屏幕上报告配置、构建和安装信息，所以选择关闭ExternalProject_Add:

.. code-block:: bash

    DOWNLOAD_NO_PROGRESS
        1
    LOG_CONFIGURE
        1
    LOG_BUILD
        1
    LOG_INSTALL
        1
  )

8 为了确保子项目在超级构建的其余部分中是可见的，我们设置了message_DIR目录:

.. code-block:: cmake

  if(WIN32 AND NOT CYGWIN)
      set(DEF_message_DIR ${STAGED_INSTALL_PREFIX}/CMake)
  else()
      set(DEF_message_DIR ${STAGED_INSTALL_PREFIX}/share/cmake/message)
  endif()
  file(TO_NATIVE_PATH "${DEF_message_DIR}" DEF_message_DIR)
  set(message_DIR ${DEF_message_DIR}
  CACHE PATH "Path to internally built messageConfig.cmake" FORCE)

最后，来看一下src目录上的CMakeLists.txt：

1 同样，声明一个C++11项目:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
  project(recipe-05_core
  LANGUAGES CXX
  )
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 项目需要消息库:

.. code-block:: cmake

  find_package(message 1 CONFIG REQUIRED)
  get_property(_loc TARGET message::message-shared PROPERTY LOCATION)
  message(STATUS "Found message: ${_loc} (found version ${message_VERSION})")

3 声明一个可执行目标，并将其链接到消息动态库:

.. code-block:: cmake

  add_executable(use_message use_message.cpp)
  target_link_libraries(use_message
    PUBLIC
        message::message-shared
  )


.. highlight:: c++

.. default-domain:: cpp

============================
第13章 选择生成器和交叉编译
============================

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-13/recipe-01 中找到，其中包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

早期版本的Visual Studio要求开发人员在不同的Windows版本中编辑源代码并运行CMake命令，
但Visual Studio 2017引入了对CMake项目的内置支持( https://aka.ms/cmake )，它允许在同一个IDE中发生整个编码、配置、构建和测试工作流。本示例中，
不需要使用命令行，我们将直接使用Visual Studio 2017构建一个简单的“hello world”CMake示例项目。


**具体实施**

创建相应的源码：

1 创建一个目录，并将hello-world.cpp放在新目录中。

2 目录中，创建一个CMakeLists.txt文件，其内容为:

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name and language
  project(recipe-01 LANGUAGES CXX)
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
  # define executable and its source file
  add_executable(hello-world hello-world.cpp)
  # we will print the system name in the code
  target_compile_definitions(hello-world
    PUBLIC
      "SYSTEM_NAME=\"${CMAKE_SYSTEM_NAME}\""
    )
  install(
    TARGETS
      hello-world
    DESTINATION
      ${CMAKE_INSTALL_BINDIR}
    )

3 打开Visual Studio 2017，然后通过下面的File ->Open -> Folder，选择到新创建的包含源文件和CMakeLists.txt的文件夹下。

4 打开文件夹后，请注意CMake配置步骤是如何运行的(面板底部)：

5 13.1 使用CMake构建Visual Studio 2017项目

6 现在，可以右键单击CMakeLists.txt(右面板)，并选择Build:

7 13.1 使用CMake构建Visual Studio 2017项目 

8 构建项目(参见底部面板上的输出):

9 13.1 使用CMake构建Visual Studio 2017项目 - 图5

这就成功地编译了可执行文件。下一小节中，我们将学习如何定位可执行文件，并更改构建和安装路径。


13.1 使用CMake构建Visual Studio 2017项目
---------------------------------------------------

**具体实施**

我们将按照以下步骤，在这个交叉编译的“hello world”示例中创建三个文件:

1 创建一个文件夹，其中包括hello-world.cpp和CMakeLists.txt。

2 再创建一个toolchain.cmake文件，其内容为：

.. code-block:: cmake

  # the name of the target operating system
  set(CMAKE_SYSTEM_NAME Windows)
  # which compilers to use
  set(CMAKE_CXX_COMPILER i686-w64-mingw32-g++)
  # adjust the default behaviour of the find commands:
  # search headers and libraries in the target environment
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  # search programs in the host environment
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

3 将CMAKE_CXX_COMPILER设置为对应的编译器(路径)。

4 然后，通过将CMAKE_TOOLCHAIN_FILE指向工具链文件，从而配置代码(本例中，使用了从源代码构建的MXE编译器):

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake -D CMAKE_TOOLCHAIN_FILE=toolchain.cmake ..
  -- The CXX compiler identification is GNU 5.4.0
  -- Check for working CXX compiler: /home/user/mxe/usr/bin/i686-w64-mingw32.static-g++
  -- Check for working CXX compiler: /home/user/mxe/usr/bin/i686-w64-mingw32.static-g++ -- works
  -- Detecting CXX compiler ABI info
  -- Detecting CXX compiler ABI info - done
  -- Detecting CXX compile features
  -- Detecting CXX compile features - done
  -- Configuring done
  -- Generating done
  -- Build files have been written to: /home/user/cmake-recipes/chapter-13/recipe-01/cxx-example/build

5 现在，构建可执行文件：

.. code-block:: bash

  $ cmake --build .
  Scanning dependencies of target hello-world
  [ 50%] Building CXX object CMakeFiles/hello-world.dir/hello-world.cpp.obj
  [100%] Linking CXX executable bin/hello-world.exe
  [100%] Built target hello-world

6 注意，我们已经在Linux上获得hello-world.exe。将二进制文件复制到Windows上。

7 在WIndows上可以看到如下的输出：

.. code-block:: bash

  Hello from Windows

8 如你所见，这个二进制可以在Windows下工作。


13.2 交叉编译hello world示例
---------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-13/recipe-01 中找到，其中包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

这个示例中，我们将重用“Hello World”示例，并将代码从Linux或macOS交叉编译到Windows。换句话说，我们将在Linux或macOS上配置和编译代码，
并生成Windows平台的可执行文件


13.3 使用OpenMP并行化交叉编译Windows二进制文件
---------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-13/recipe-02 中找到，其中包含一个C++示例和Fortran示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

在这个示例中，我们将交叉编译一个OpenMP并行化的Windows二进制文件。

**具体实施**

通过以下步骤，我们将设法交叉编译一个OpenMP并行化的Windows可执行文件:

1 创建一个包含example.cpp和CMakeLists.txt的目录。

2 我们将使用与之前例子相同的toolchain.cmake:

.. code-block:: cmake

  # the name of the target operating system
  set(CMAKE_SYSTEM_NAME Windows)
  # which compilers to use
  set(CMAKE_CXX_COMPILER i686-w64-mingw32-g++)
  # adjust the default behaviour of the find commands:
  # search headers and libraries in the target environment
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  # search programs in the host environment
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

3 将CMAKE_CXX_COMPILER设置为对应的编译器(路径)。

4 然后，通过CMAKE_TOOLCHAIN_FILE指向工具链文件来配置代码(本例中，使用了从源代码构建的MXE编译器):

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake -D CMAKE_TOOLCHAIN_FILE=toolchain.cmake ..
  -- The CXX compiler identification is GNU 5.4.0
  -- Check for working CXX compiler: /home/user/mxe/usr/bin/i686-w64-mingw32.static-g++
  -- Check for working CXX compiler: /home/user/mxe/usr/bin/i686-w64-mingw32.static-g++ -- works
  -- Detecting CXX compiler ABI info
  -- Detecting CXX compiler ABI info - done
  -- Detecting CXX compile features
  -- Detecting CXX compile features - done
  -- Found OpenMP_CXX: -fopenmp (found version "4.0")
  -- Found OpenMP: TRUE (found version "4.0")
  -- Configuring done
  -- Generating done
  -- Build files have been written to: /home/user/cmake-recipes/chapter-13/recipe-02/cxx-example/build

5 构建可执行文件：

.. code-block:: bash

  $ cmake --build .
  Scanning dependencies of target example
  [ 50%] Building CXX object CMakeFiles/example.dir/example.cpp.obj
  [100%] Linking CXX executable bin/example.exe
  [100%] Built target example

6 将example.exe拷贝到Windows环境下。

7 Windows环境下，将看到如下的输出：

.. code-block:: bash

  $ set OMP_NUM_THREADS=1
  $ example.exe 1000000000
  number of available processors: 2
  number of threads: 1
  we will form sum of numbers from 1 to 1000000000
  sum: 500000000500000000
  elapsed wall clock time: 2.641 seconds
  $ set OMP_NUM_THREADS=2
  $ example.exe 1000000000
  number of available processors: 2
  number of threads: 2
  we will form sum of numbers from 1 to 1000000000
  sum: 500000000500000000
  elapsed wall clock time: 1.328 seconds

8 正如我们所看到的，二进制文件可以在Windows上工作，而且由于OpenMP并行化，我们可以观察到加速效果!

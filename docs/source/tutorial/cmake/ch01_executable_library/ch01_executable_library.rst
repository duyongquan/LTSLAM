.. highlight:: c++

.. default-domain:: cpp

==========================
第1章 从可执行文件到库
==========================

本章的主要内容有：

* 将单个源码文件编译为可执行文件
* 切换生成器
* 构建和连接静态库与动态库
* 用条件语句控制编译
* 向用户显示选项
* 指定编译器
* 切换构建类型
* 设置编译器选项
* 为语言设定标准
* 使用控制流进行构造


1.1 将单个源文件编译为可执行文件
-----------------------------------

本节示例中，我们将演示如何运行CMake配置和构建一个简单的项目。该项目由单个源文件组成，用于生成可执行文件。
我们将用C++讨论这个项目，您在GitHub示例库中可以找到C和Fortran的例子。


**准备工作**

我们希望将以下源代码编译为单个可执行文件：

.. code-block:: c++

    #include <cstdlib>
    #include <iostream>
    #include <string>

    std::string say_hello() 
    { 
      return std::string("Hello, CMake world!"); 
    }

    int main() {
      std::cout << say_hello() << std::endl;
      return EXIT_SUCCESS;
    }


**具体实施**

除了源文件之外，我们还需要向CMake提供项目配置描述。该描述使用CMake完成，完整的文档可以在 https://cmake.org/cmake/help/latest/ 找到。我们把CMake指令放入一个名为CMakeLists.txt的文件中。

.. NOTE:: 
  
  文件的名称区分大小写，必须命名为CMakeLists.txt，CMake才能够解析。

**具体步骤如下：**

用编辑器打开一个文本文件，将这个文件命名为CMakeLists.txt。

第一行，设置CMake所需的最低版本。如果使用的CMake版本低于该版本，则会发出致命错误：

.. code-block:: bash

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

第二行，声明了项目的名称(recipe-01)和支持的编程语言(CXX代表C++)：

.. code-block:: bash

  project(recipe-01 LANGUAGES CXX)

指示CMake创建一个新目标：可执行文件hello-world。这个可执行文件是通过编译和链接源文件hello-world.cpp生成的。CMake将为编译器使用默认设置，并自动选择生成工具：

.. code-block:: bash

  add_executable(hello-world hello-world.cpp)

将该文件与源文件hello-world.cpp放在相同的目录中。记住，它只能被命名为CMakeLists.txt。

现在，可以通过创建build目录，在build目录下来配置项目：

.. code-block:: bash

  mkdir -p build
  cd build
  cmake ..

  -- The CXX compiler identification is GNU 8.1.0
  -- Check for working CXX compiler: /usr/bin/c++
  -- Check for working CXX compiler: /usr/bin/c++ -- works
  -- Detecting CXX compiler ABI info
  -- Detecting CXX compiler ABI info - done
  -- Detecting CXX compile features
  -- Detecting CXX compile features - done
  -- Configuring done
  -- Generating done
  -- Build files have been written to: /home/user/cmake-cookbook/chapter-01/recipe-01/cxx-example/build


如果一切顺利，项目的配置已经在build目录中生成。我们现在可以编译可执行文件：

.. code-block:: bash

  cmake --build .
  Scanning dependencies of target hello-world
  [ 50%] Building CXX object CMakeFiles/hello-world.dir/hello-world.cpp.o
  [100%] Linking CXX executable hello-world
  [100%] Built target hello-world

**工作原理**

示例中，我们使用了一个简单的CMakeLists.txt来构建“Hello world”可执行文件：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES CXX)
  add_executable(hello-world hello-world.cpp)


.. NOTE::

  CMake语言不区分大小写，但是参数区分大小写。

  CMake中，C++是默认的编程语言。不过，我们还是建议使用LANGUAGES选项在project命令中显式地声明项目的语言。


要配置项目并生成构建器，我们必须通过命令行界面(CLI)运行CMake。CMake CLI提供了许多选项，cmake -help将输出以显示列出所有可用选项的完整帮助信息，
我们将在书中对这些选项进行更多地了解。正如您将从cmake -help的输出中显示的内容，它们中的大多数选项会让你您访问CMake手册，查看详细信息。
通过下列命令生成构建器：

.. code-block:: bash

  mkdir -p build
  cd build
  cmake ..

这里，我们创建了一个目录build(生成构建器的位置)，进入build目录，并通过指定CMakeLists.txt的位置(本例中位于父目录中)来调用CMake。
可以使用以下命令行来实现相同的效果：

.. code-block:: bash

  cmake -H. -Bbuild

该命令是跨平台的，使用了-H和-B为CLI选项。-H表示当前目录中搜索根CMakeLists.txt文件。-Bbuild告诉CMake在一个名为build的目录中生成所有的文件。

运行cmake命令会输出一系列状态消息，显示配置信息：

.. code-block:: bash

  cmake ..
  -- The CXX compiler identification is GNU 8.1.0
  -- Check for working CXX compiler: /usr/bin/c++
  -- Check for working CXX compiler: /usr/bin/c++ -- works
  -- Detecting CXX compiler ABI info
  -- Detecting CXX compiler ABI info - done
  -- Detecting CXX compile features
  -- Detecting CXX compile features - done
  -- Configuring done
  -- Generating done
  -- Build files have been written to: /home/user/cmake-cookbook/chapter-01/recipe-01/cxx-example/build

.. NOTE::

  在与CMakeLists.txt相同的目录中执行cmake .，原则上足以配置一个项目。然而，CMake会将所有生成的文件写到项目的根目录中。这将是一个源代码内构建，通常是不推荐的，因为这会混合源代码和项目的目录树。我们首选的是源外构建。

CMake是一个构建系统生成器。将描述构建系统(如：Unix Makefile、Ninja、Visual Studio等)应当如何操作才能编译代码。然后，CMake为所选的构建系统生成相应的指令。默认情况下，在GNU/Linux和macOS系统上，CMake使用Unix Makefile生成器。Windows上，Visual Studio是默认的生成器。
在下一个示例中，我们将进一步研究生成器，并在第13章中重新讨论生成器。


GNU/Linux上，CMake默认生成Unix Makefile来构建项目：

* Makefile: make将运行指令来构建项目。
* CMakefile：包含临时文件的目录，CMake用于检测操作系统、编译器等。此外，根据所选的生成器，它还包含特定的文件。
* cmake_install.cmake：处理安装规则的CMake脚本，在项目安装时使用。
* CMakeCache.txt：如文件名所示，CMake缓存。CMake在重新运行配置时使用这个文件。

要构建示例项目，我们运行以下命令：

.. code-block:: bash

  cmake --build .

最后，CMake不强制指定构建目录执行名称或位置，我们完全可以把它放在项目路径之外。这样做同样有效：

.. code-block:: bash

  mkdir -p /tmp/someplace
  cd /tmp/someplace
  cmake /path/to/source
  cmake --build .

CMake生成的目标比构建可执行文件的目标要多。可以使用cmake --build . --target <target-name>语法，实现如下功能

* all(或Visual Studio generator中的ALL_BUILD)是默认目标，将在项目中构建所有目标。
* clean，删除所有生成的文件。
* rebuild_cache，将调用CMake为源文件生成依赖(如果有的话)。
* edit_cache，这个目标允许直接编辑缓存。

对于更复杂的项目，通过测试阶段和安装规则，CMake将生成额外的目标：

*  test(或Visual Studio generator中的RUN_TESTS)将在CTest的帮助下运行测试套件。我们将在第4章中详细讨论测试和CTest。
* install，将执行项目安装规则。我们将在第10章中讨论安装规则。
* package，此目标将调用CPack为项目生成可分发的包。打包和CPack将在第11章中讨论。


1.2 切换生成器
----------------------

1.3 构建和链接静态库和动态库
----------------------------

1.4 用条件句控制编译
----------------------

1.5 向用户显示选项
----------------------

1.6 指定编译器
----------------------

1.7 切换构建类型
----------------------

1.8 设置编译器选项
----------------------

目前为止，我们还没有过多考虑如何选择编译器。CMake可以根据平台和生成器选择编译器，还能将编译器标志设置为默认值。
然而，我们通常控制编译器的选择。在后面的示例中，我们还将考虑构建类型的选择，并展示如何控制编译器标志。

**具体实施**

如何选择一个特定的编译器？例如，如果想使用Intel或Portland Group编译器怎么办？CMake将语言的编译器存储在CMAKE_<LANG>_COMPILER变量中，
其中<LANG>是受支持的任何一种语言，对于我们的目的是CXX、C或Fortran。用户可以通过以下两种方式之一设置此变量：

使用CLI中的-D选项，例如：

.. code-block:: bash

  cmake -D CMAKE_CXX_COMPILER=clang++ ..

通过导出环境变量CXX(C++编译器)、CC(C编译器)和FC(Fortran编译器)。例如，使用这个命令使用clang++作为C++编译器：

.. code-block:: bash

  env CXX=clang++ cmake ..

到目前为止讨论的示例，都可以通过传递适当的选项，配置合适的编译器。

.. NOTE:: 

  CMake了解运行环境，可以通过其CLI的-D开关或环境变量设置许多选项。前一种机制覆盖后一种机制，但是我们建议使用-D显式设置选项。显式优于隐式，因为环境变量可能被设置为不适合(当前项目)的值。
  我们在这里假设，其他编译器在标准路径中可用，CMake在标准路径中执行查找编译器。如果不是这样，用户将需要将完整的编译器可执行文件或包装器路径传递给CMake。

.. NOTE::

  我们建议使用-D CMAKE_<LANG>_COMPILERCLI选项设置编译器，而不是导出CXX、CC和FC。这是确保跨平台并与非POSIX兼容的唯一方法。为了避免变量污染环境，这些变量可能会影响与项目一起构建的外部库环境。

**工作原理**

配置时，CMake会进行一系列平台测试，以确定哪些编译器可用，以及它们是否适合当前的项目。一个合适的编译器不仅取决于我们所使用的平台，
还取决于我们想要使用的生成器。CMake执行的第一个测试基于项目语言的编译器的名称。例如，cc是一个工作的C编译器，那么它将用作C项目的默认编译器。
GNU/Linux上，使用Unix Makefile或Ninja时, GCC家族中的编译器很可能是C++、C和Fortran的默认选择。Microsoft Windows上，
将选择Visual Studio中的C++和C编译器(前提是Visual Studio是生成器)。如果选择MinGW或MSYS Makefile作为生成器，则默认使用MinGW编译器。

**更多信息**

我们的平台上的CMake，在哪里可以找到可用的编译器和编译器标志？CMake提供--system-information标志，它将把关于系统的所有信息转储到屏幕或文件中。要查看这个信息，请尝试以下操作：

.. code-block:: bash

  cmake --system-information information.txt

文件中(本例中是information.txt)可以看到CMAKE_CXX_COMPILER、CMAKE_C_COMPILER和CMAKE_Fortran_COMPILER的默认值，以及默认标志。我们将在下一个示例中看到相关的标志。

CMake提供了额外的变量来与编译器交互：

* CMAKE_<LANG>_COMPILER_LOADED:如果为项目启用了语言<LANG>，则将设置为TRUE。
* CMAKE_<LANG>_COMPILER_ID:编译器标识字符串，编译器供应商所特有。例如，GCC用于GNU编译器集合，AppleClang用于macOS上的Clang, MSVC用于Microsoft Visual Studio编译器。注意，不能保证为所有编译器或语言定义此变量。
* CMAKE_COMPILER_IS_GNU<LANG>:如果语言<LANG>是GNU编译器集合的一部分，则将此逻辑变量设置为TRUE。注意变量名的<LANG>部分遵循GNU约定：C语言为CC, C++语言为CXX, Fortran语言为G77。
* CMAKE_<LANG>_COMPILER_VERSION:此变量包含一个字符串，该字符串给定语言的编译器版本。版本信息在major[.minor[.patch[.tweak]]]中给出。但是，对于CMAKE_<LANG>_COMPILER_ID，不能保证所有编译器或语言都定义了此变量。

我们可以尝试使用不同的编译器，配置下面的示例CMakeLists.txt。这个例子中，我们将使用CMake变量来探索已使用的编译器(及版本)：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-06 LANGUAGES C CXX)
  message(STATUS "Is the C++ compiler loaded? ${CMAKE_CXX_COMPILER_LOADED}")
  if(CMAKE_CXX_COMPILER_LOADED)
      message(STATUS "The C++ compiler ID is: ${CMAKE_CXX_COMPILER_ID}")
      message(STATUS "Is the C++ from GNU? ${CMAKE_COMPILER_IS_GNUCXX}")
      message(STATUS "The C++ compiler version is: ${CMAKE_CXX_COMPILER_VERSION}")
  endif()
  message(STATUS "Is the C compiler loaded? ${CMAKE_C_COMPILER_LOADED}")
  if(CMAKE_C_COMPILER_LOADED)
      message(STATUS "The C compiler ID is: ${CMAKE_C_COMPILER_ID}")
      message(STATUS "Is the C from GNU? ${CMAKE_COMPILER_IS_GNUCC}")
      message(STATUS "The C compiler version is: ${CMAKE_C_COMPILER_VERSION}")
  endif()

注意，这个例子不包含任何目标，没有要构建的东西，我们只关注配置步骤:

.. code-block:: bash

  mkdir -p build
  cd build
  cmake ..
  ...
  -- Is the C++ compiler loaded? 1
  -- The C++ compiler ID is: GNU
  -- Is the C++ from GNU? 1
  -- The C++ compiler version is: 8.1.0
  -- Is the C compiler loaded? 1
  -- The C compiler ID is: GNU
  -- Is the C from GNU? 1
  -- The C compiler version is: 8.1.0
  ...

当然，输出将取决于可用和已选择的编译器(及版本)。

1.9 为语言设定标准
----------------------

1.10 使用控制流
----------------------


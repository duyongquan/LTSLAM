.. highlight:: c++

.. default-domain:: cpp

==========================
第3章 检测外部库和程序
==========================

3.1 检测Python解释器
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-01 中找到。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

Python是一种非常流行的语言。许多项目用Python编写的工具，从而将主程序和库打包在一起，或者在配置或构建过程中使用Python脚本。
这种情况下，确保运行时对Python解释器的依赖也需要得到满足。本示例将展示如何检测和使用Python解释器。

我们将介绍find_package命令，这个命令将贯穿本章。

**具体实施**

我们将逐步建立CMakeLists.txt文件:

1 首先，定义CMake最低版本和项目名称。注意，这里不需要任何语言支持:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES NONE)

2 然后，使用find_package命令找到Python解释器:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)

3 然后，执行Python命令并捕获它的输出和返回值:

.. code-block:: cmake

  execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} "-c" "print('Hello, world!')"
    RESULT_VARIABLE _status
    OUTPUT_VARIABLE _hello_world
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

4 最后，打印Python命令的返回值和输出:

.. code-block:: cmake

  message(STATUS "RESULT_VARIABLE is: ${_status}")
  message(STATUS "OUTPUT_VARIABLE is: ${_hello_world}")

5 配置项目:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  -- Found PythonInterp: /usr/bin/python (found version "3.6.5")
  -- RESULT_VARIABLE is: 0
  -- OUTPUT_VARIABLE is: Hello, world!
  -- Configuring done
  -- Generating done
  -- Build files have been written to: /home/user/cmake-cookbook/chapter-03/recipe-01/example/build

3.2 检测Python库
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/devcafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-02 中找到，有一个C示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

可以使用Python工具来分析和操作程序的输出。然而，还有更强大的方法可以将解释语言(如Python)与编译语言(如C或C++)组合在一起使用。一种是扩展Python，
通过编译成共享库的C或C++模块在这些类型上提供新类型和新功能，这是第9章的主题。另一种是将Python解释器嵌入到C或C++程序中。两种方法都需要下列条件:

* Python解释器的工作版本
* Python头文件Python.h的可用性
* Python运行时库libpython

三个组件所使用的Python版本必须相同。我们已经演示了如何找到Python解释器；本示例中，我们将展示另外两种方式。


**具体实施**

以下是CMakeLists.txt中的步骤:

1 包含CMake最低版本、项目名称和所需语言:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-02 LANGUAGES C)

2 制使用C99标准，这不严格要求与Python链接，但有时你可能需要对Python进行连接:

.. code-block:: cmake

  set(CMAKE_C_STANDARD 99)
  set(CMAKE_C_EXTENSIONS OFF)
  set(CMAKE_C_STANDARD_REQUIRED ON)

3 找到Python解释器。这是一个REQUIRED依赖:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)

4 找到Python头文件和库的模块，称为FindPythonLibs.cmake:

.. code-block:: cmake

  find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} EXACT REQUIRED)

5 使用hello-embedded-python.c源文件，添加一个可执行目标:

.. code-block:: cmake

  add_executable(hello-embedded-python hello-embedded-python.c)

6 可执行文件包含Python.h头文件。因此，这个目标的include目录必须包含Python的include目录，可以通过PYTHON_INCLUDE_DIRS变量进行指定:

.. code-block:: cmake

  target_include_directories(hello-embedded-python
    PRIVATE
        ${PYTHON_INCLUDE_DIRS}
      )

7 最后，将可执行文件链接到Python库，通过PYTHON_LIBRARIES变量访问:

.. code-block:: cmake

  target_link_libraries(hello-embedded-python
    PRIVATE
        ${PYTHON_LIBRARIES}
      )

8 现在，进行构建:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  ...
  -- Found PythonInterp: /usr/bin/python (found version "3.6.5")
  -- Found PythonLibs: /usr/lib/libpython3.6m.so (found suitable exact version "3.6.5")

9 最后，执行构建，并运行可执行文件:

.. code-block:: bash

  $ cmake --build .
  $ ./hello-embedded-python
  Today is Thu Jun 7 22:26:02 2018


3.3 检测Python模块和包
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/devcafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-03 中找到，包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

前面的示例中，我们演示了如何检测Python解释器，以及如何编译一个简单的C程序(嵌入Python解释器)。通常，代码将依赖于特定的Python模块，无论是Python工具、
嵌入Python的程序，还是扩展Python的库。例如，科学界非常流行使用NumPy处理矩阵问题。依赖于Python模块或包的项目中，确定满足对这些Python模块的依赖非常重要。
本示例将展示如何探测用户的环境，以找到特定的Python模块和包。

**具体实施**

下面的代码中，我们能够使用CMake检查NumPy是否可用。我们需要确保Python解释器、头文件和库在系统上是可用的。然后，将再来确认NumPy的可用性：

1 首先，我们定义了最低CMake版本、项目名称、语言和C++标准:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-03 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 查找解释器、头文件和库的方法与前面的方法完全相同:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)
  find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} EXACT REQUIRED)

3 正确打包的Python模块，指定安装位置和版本。可以在CMakeLists.txt中执行Python脚本进行探测:

.. code-block:: cmake

  execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} "-c" "import re, numpy; print(re.compile('/__init__.py.*').sub('',numpy.__file__))"
    RESULT_VARIABLE _numpy_status
    OUTPUT_VARIABLE _numpy_location
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

4 如果找到NumPy，则_numpy_status变量为整数，否则为错误的字符串，而_numpy_location将包含NumPy模块的路径。如果找到NumPy，
则将它的位置保存到一个名为NumPy的新变量中。注意，新变量被缓存，这意味着CMake创建了一个持久性变量，用户稍后可以修改该变量:

.. code-block:: cmake

  if(NOT _numpy_status)
      set(NumPy ${_numpy_location} CACHE STRING "Location of NumPy")
  endif()

5 下一步是检查模块的版本。同样，我们在CMakeLists.txt中施加了一些Python魔法，将版本保存到_numpy_version变量中:

.. code-block:: cmake

  execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} "-c" "import numpy; print(numpy.__version__)"
    OUTPUT_VARIABLE _numpy_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

6 最后，FindPackageHandleStandardArgs的CMake包以正确的格式设置NumPy_FOUND变量和输出信息:

.. code-block:: cmake

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(NumPy
    FOUND_VAR NumPy_FOUND
    REQUIRED_VARS NumPy
    VERSION_VAR _numpy_version
    )

7 一旦正确的找到所有依赖项，我们就可以编译可执行文件，并将其链接到Python库:

.. code-block:: cmake

  add_executable(pure-embedding "")
  target_sources(pure-embedding
    PRIVATE
        Py${PYTHON_VERSION_MAJOR}-pure-embedding.cpp
    )
  target_include_directories(pure-embedding
    PRIVATE
        ${PYTHON_INCLUDE_DIRS}
    )
  target_link_libraries(pure-embedding
    PRIVATE
        ${PYTHON_LIBRARIES}
    )

8 我们还必须保证use_numpy.py在build目录中可用:

.. code-block:: cmake

  add_custom_command(
    OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/use_numpy.py
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/use_numpy.py
        ${CMAKE_CURRENT_BINARY_DIR}/use_numpy.py
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/use_numpy.py
    )
  # make sure building pure-embedding triggers the above custom command
  target_sources(pure-embedding
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/use_numpy.py
    )

9 现在，我们可以测试嵌入的代码:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  -- ...
  -- Found PythonInterp: /usr/bin/python (found version "3.6.5")
  -- Found PythonLibs: /usr/lib/libpython3.6m.so (found suitable exact version "3.6.5")
  -- Found NumPy: /usr/lib/python3.6/site-packages/numpy (found version "1.14.3")
  $ cmake --build .
  $ ./pure-embedding use_numpy print_ones 2 3
  [[1. 1. 1.]
  [1. 1. 1.]]
  Result of call: 6


3.4 检测BLAS和LAPACK数学库
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-04 中找到，有一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

许多数据算法严重依赖于矩阵和向量运算。例如：矩阵-向量和矩阵-矩阵乘法，求线性方程组的解，特征值和特征向量的计算或奇异值分解。这些操作在代码库中非常普遍，
因为操作的数据量比较大，因此高效的实现有绝对的必要。幸运的是，有专家库可用：基本线性代数子程序(BLAS)和线性代数包(LAPACK)，为许多线性代数操作提供了标准API。
供应商有不同的实现，但都共享API。虽然，用于数学库底层实现，实际所用的编程语言会随着时间而变化(Fortran、C、Assembly)，但是也都是Fortran调用接口。
考虑到调用街扩，本示例中的任务要链接到这些库，并展示如何用不同语言编写的库。

**具体实施**

对应的CMakeLists.txt包含以下构建块:

1 我们定义了CMake最低版本，项目名称和支持的语言:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-04 LANGUAGES CXX C Fortran)

2 使用C++11标准:

.. code-block:: cmake

  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

3 此外，我们验证Fortran和C/C++编译器是否能协同工作，并生成头文件，这个文件可以处理名称混乱。两个功能都由FortranCInterface模块提供:

.. code-block:: cmake

  include(FortranCInterface)
  FortranCInterface_VERIFY(CXX)
  FortranCInterface_HEADER(
    fc_mangle.h
    MACRO_NAMESPACE "FC_"
    SYMBOLS DSCAL DGESV
    )

4 然后，找到BLAS和LAPACK:

.. code-block:: cmake

  find_package(BLAS REQUIRED)
  find_package(LAPACK REQUIRED)

5 接下来，添加一个库，其中包含BLAS和LAPACK包装器的源代码，并链接到LAPACK_LIBRARIES，其中也包含BLAS_LIBRARIES:

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

6 注意，目标的包含目录和链接库声明为PUBLIC，因此任何依赖于数学库的附加目标也将在其包含目录中。

7 最后，我们添加一个可执行目标并链接math：

.. code-block:: cmake

  add_executable(linear-algebra "")
  target_sources(linear-algebra
    PRIVATE
        linear-algebra.cpp
    )
  target_link_libraries(linear-algebra
    PRIVATE
        math
    )

8 配置时，我们可以关注相关的打印输出:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  ...
  -- Detecting Fortran/C Interface
  -- Detecting Fortran/C Interface - Found GLOBAL and MODULE mangling
  -- Verifying Fortran/C Compiler Compatibility
  -- Verifying Fortran/C Compiler Compatibility - Success
  ...
  -- Found BLAS: /usr/lib/libblas.so
  ...
  -- A library with LAPACK API found.
  ...

9 最后，构建并测试可执行文件:

.. code-block:: bash

$ cmake --build .
$ ./linear-algebra 1000
C_DSCAL done
C_DGESV done
info is 0
check is 1.54284e-10


3.5 检测OpenMP的并行环境
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-05 中找到，有一个C++和一个Fortran示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。
  https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-05 中也有一个适用于CMake 3.5的示例。

目前，市面上的计算机几乎都是多核机器，对于性能敏感的程序，我们必须关注这些多核处理器，并在编程模型中使用并发。OpenMP是多核处理器上并行性的标准之一。
为了从OpenMP并行化中获得性能收益，通常不需要修改或重写现有程序。一旦确定了代码中的性能关键部分，例如：使用分析工具，程序员就可以通过预处理器指令，
指示编译器为这些区域生成可并行的代码。

本示例中，我们将展示如何编译一个包含OpenMP指令的程序(前提是使用一个支持OpenMP的编译器)。有许多支持OpenMP的Fortran、C和C++编译器。对于相对较新的CMake版本，
为OpenMP提供了非常好的支持。本示例将展示如何在使用CMake 3.9或更高版本时，使用简单C++和Fortran程序来链接到OpenMP。

**具体实施**

对于C++和Fortran的例子，CMakeLists.txt将遵循一个模板，该模板在这两种语言上很相似：

1 两者都定义了CMake最低版本、项目名称和语言(CXX或Fortran；我们将展示C++版本):

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
  project(recipe-05 LANGUAGES CXX)

2 使用C++11标准:

.. code-block:: cmake

  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

3 调用find_package来搜索OpenMP:

.. code-block:: cmake

  find_package(OpenMP REQUIRED)

4 最后，我们定义可执行目标，并链接到FindOpenMP模块提供的导入目标(在Fortran的情况下，我们链接到OpenMP::OpenMP_Fortran):

.. code-block:: cmake

  add_executable(example example.cpp)
  target_link_libraries(example
    PUBLIC
        OpenMP::OpenMP_CXX
    )

5 现在，可以配置和构建代码了:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .

6 并行测试(在本例中使用了4个内核):

.. code-block:: bash

  $ ./example 1000000000
  number of available processors: 4
  number of threads: 4
  we will form sum of numbers from 1 to 1000000000
  sum: 500000000500000000
  elapsed wall clock time: 1.08343 seconds

7 为了比较，我们可以重新运行这个例子，并将OpenMP线程的数量设置为1:

.. code-block:: bash

  $ env OMP_NUM_THREADS=1 ./example 1000000000
  number of available processors: 4
  number of threads: 1
  we will form sum of numbers from 1 to 1000000000
  sum: 500000000500000000
  elapsed wall clock time: 2.96427 seconds


3.6 检测MPI的并行环境
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-06 中找到，包含一个C++和一个C的示例。
  该示例在CMake 3.9版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。
  https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-06 中也有一个适用于CMake 3.5的C示例。


消息传递接口(Message Passing Interface, MPI)，可以作为OpenMP(共享内存并行方式)的补充，它也是分布式系统上并行程序的实际标准。
尽管，最新的MPI实现也允许共享内存并行，但高性能计算中的一种典型方法就是，在计算节点上OpenMP与MPI结合使用。MPI标准的实施包括:

* 运行时库
* 头文件和Fortran 90模块
* 编译器的包装器，用来调用编译器，使用额外的参数来构建MPI库，以处理目录和库。通常，包装器mpic++/mpiCC/mpicxx用于C++，mpicc用于C，mpifort用于Fortran。
* 启动MPI：应该启动程序，以编译代码的并行执行。它的名称依赖于实现，可以使用这几个命令启动：mpirun、mpiexec或orterun。

本示例，将展示如何在系统上找到合适的MPI实现，从而编译一个简单的“Hello, World”MPI例程。

**具体实施**

这个示例中，我们先查找MPI实现：库、头文件、编译器包装器和启动器。为此，我们将用到FindMPI.cmake标准CMake模块:

1 首先，定义了CMake最低版本、项目名称、支持的语言和语言标准:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
  project(recipe-06 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 然后，调用find_package来定位MPI:

.. code-block:: cmake

  find_package(MPI REQUIRED)

3 与前面的配置类似，定义了可执行文件的的名称和相关源码，并链接到目标:

.. code-block:: cmake

  add_executable(hello-mpi hello-mpi.cpp)
  target_link_libraries(hello-mpi
    PUBLIC
        MPI::MPI_CXX
    )

4 配置和构建可执行文件:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake .. # -D CMAKE_CXX_COMPILER=mpicxx C++例子中可加，加与不加对于构建结果没有影响╭(╯^╰)╮
  -- ...
  -- Found MPI_CXX: /usr/lib/openmpi/libmpi_cxx.so (found version "3.1")
  -- Found MPI: TRUE (found version "3.1")
  -- ...
  $ cmake --build .

5 为了并行执行这个程序，我们使用mpirun启动器(本例中，启动了两个任务):

.. code-block:: bash

  $ mpirun -np 2 ./hello-mpi
  Hello world from processor larry, rank 1 out of 2 processors
  Hello world from processor larry, rank 0 out of 2 processors


3.7 检测Eigen库
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-07 中找到，包含一个C++的示例。该示例在CMake 3.9版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-06 中也有一个适用于CMake 3.5的C++示例。

BLAS库为矩阵和向量操作提供了标准化接口。不过，这个接口用Fortran语言书写。虽然已经展示了如何使用C++直接使用这些库，但在现代C++程序中，希望有更高级的接口。

纯头文件实现的Eigen库，使用模板编程来提供接口。矩阵和向量的计算，会在编译时进行数据类型检查，以确保兼容所有维度的矩阵。密集和稀疏矩阵的运算，也可使用表达式模板高效的进行实现，如：矩阵-矩阵乘积，线性系统求解器和特征值问题。从3.3版开始，Eigen可以链接到BLAS和LAPACK库中，这可以将某些操作实现进行卸载，使库的实现更加灵活，从而获得更多的性能收益。

本示例将展示如何查找Eigen库，使用OpenMP并行化，并将部分工作转移到BLAS库。

本示例中会实现，矩阵-向量乘法和LU分解)，可以选择卸载BLAS和LAPACK库中的一些实现。这个示例中，只考虑将在BLAS库中卸载。

**准备工作**

本例中，我们编译一个程序，该程序会从命令行获取的随机方阵和维向量。然后我们将用LU分解来解线性方程组Ax=b。以下是源代码(linear-algebra.cpp):

.. code-block:: bash

  #include <chrono>
  #include <cmath>
  #include <cstdlib>
  #include <iomanip>
  #include <iostream>
  #include <vector>
  #include <Eigen/Dense>

  int main(int argc, char **argv)
  {
    if (argc != 2)
    {
      std::cout << "Usage: ./linear-algebra dim" << std::endl;
      return EXIT_FAILURE;
    }
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;
    std::time_t end_time;
    std::cout << "Number of threads used by Eigen: " << Eigen::nbThreads()
              << std::endl;
    // Allocate matrices and right-hand side vector
    start = std::chrono::system_clock::now();
    int dim = std::atoi(argv[1]);
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(dim, dim);
    Eigen::VectorXd b = Eigen::VectorXd::Random(dim);
    end = std::chrono::system_clock::now();
    // Report times
    elapsed_seconds = end - start;
    end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "matrices allocated and initialized "
              << std::put_time(std::localtime(&end_time), "%a %b %d %Y
    %r\n")
              << "elapsed time: " << elapsed_seconds.count() << "s\n";
    start = std::chrono::system_clock::now();
    // Save matrix and RHS
    Eigen::MatrixXd A1 = A;
    Eigen::VectorXd b1 = b;
    end = std::chrono::system_clock::now();
    end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "Scaling done, A and b saved "
              << std::put_time(std::localtime(&end_time), "%a %b %d %Y %r\n")
              << "elapsed time: " << elapsed_seconds.count() << "s\n";
    start = std::chrono::system_clock::now();
    Eigen::VectorXd x = A.lu().solve(b);
    end = std::chrono::system_clock::now();
    // Report times
    elapsed_seconds = end - start;
    end_time = std::chrono::system_clock::to_time_t(end);
    double relative_error = (A * x - b).norm() / b.norm();
    std::cout << "Linear system solver done "
              << std::put_time(std::localtime(&end_time), "%a %b %d %Y %r\n")
              << "elapsed time: " << elapsed_seconds.count() << "s\n";
    std::cout << "relative error is " << relative_error << std::endl;
    return 0;
  }

矩阵-向量乘法和LU分解是在Eigen库中实现的，但是可以选择BLAS和LAPACK库中的实现。在这个示例中，我们只考虑BLAS库中的实现。

**具体实施**

这个示例中，我们将用到Eigen和BLAS库，以及OpenMP。使用OpenMP将Eigen并行化，并从BLAS库中卸载部分线性代数实现:

1 首先声明CMake最低版本、项目名称和使用C++11语言标准:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
  project(recipe-07 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 因为Eigen可以使用共享内存的方式，所以可以使用OpenMP并行处理计算密集型操作:

.. code-block:: cmake

  find_package(OpenMP REQUIRED)

3 调用find_package来搜索Eigen(将在下一小节中讨论):

.. code-block:: cmake

  find_package(Eigen3 3.3 REQUIRED CONFIG)

4 如果找到Eigen，我们将打印状态信息。注意，使用的是Eigen3::Eigen，这是一个IMPORT目标，可通过提供的CMake脚本找到这个目标:

.. code-block:: cmake

  if(TARGET Eigen3::Eigen)
    message(STATUS "Eigen3 v${EIGEN3_VERSION_STRING} found in ${EIGEN3_INCLUDE_DIR}")
  endif()

5 接下来，将源文件声明为可执行目标:

.. code-block:: cmake

  add_executable(linear-algebra linear-algebra.cpp)

6 然后，找到BLAS。注意，现在不需要依赖项:

.. code-block:: cmake

  find_package(BLAS)

7 如果找到BLAS，我们可为可执行目标，设置相应的宏定义和链接库:

.. code-block:: cmake

  if(BLAS_FOUND)
    message(STATUS "Eigen will use some subroutines from BLAS.")
    message(STATUS "See: http://eigen.tuxfamily.org/dox-devel/TopicUsingBlasLapack.html")
    target_compile_definitions(linear-algebra
      PRIVATE
          EIGEN_USE_BLAS
      )
    target_link_libraries(linear-algebra
      PUBLIC
          ${BLAS_LIBRARIES}
      )
  else()
      message(STATUS "BLAS not found. Using Eigen own functions")
  endif()

8 最后，我们链接到Eigen3::Eigen和OpenMP::OpenMP_CXX目标。这就可以设置所有必要的编译标示和链接标志:

.. code-block:: cmake

  target_link_libraries(linear-algebra
    PUBLIC
      Eigen3::Eigen
      OpenMP::OpenMP_CXX
    )

9 开始配置:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  -- ...
  -- Found OpenMP_CXX: -fopenmp (found version "4.5")
  -- Found OpenMP: TRUE (found version "4.5")
  -- Eigen3 v3.3.4 found in /usr/include/eigen3
  -- ...
  -- Found BLAS: /usr/lib/libblas.so
  -- Eigen will use some subroutines from BLAS.
  -- See: http://eigen.tuxfamily.org/dox-devel/TopicUsingBlasLapack.html

10 最后，编译并测试代码。注意，可执行文件使用四个线程运行:

.. code-block:: bash

  $ cmake --build .
  $ ./linear-algebra 1000
  Number of threads used by Eigen: 4
  matrices allocated and initialized Sun Jun 17 2018 11:04:20 AM
  elapsed time: 0.0492328s
  Scaling done, A and b saved Sun Jun 17 2018 11:04:20 AM
  elapsed time: 0.0492328s
  Linear system solver done Sun Jun 17 2018 11:04:20 AM
  elapsed time: 0.483142s
  relative error is 4.21946e-13

**工作原理**

Eigen支持CMake查找，这样配置项目就会变得很容易。从3.3版开始，Eigen提供了CMake模块，这些模块将导出相应的目标Eigen3::Eigen。

find_package可以通过选项传递，届时CMake将不会使用FindEigen3.cmake模块，而是通过特定的Eigen3Config.cmake，Eigen3ConfigVersion.cmake和Eigen3Targets.cmake提供Eigen3安装的标准位置(<installation-prefix>/share/eigen3/cmake)。这种包定位模式称为“Config”模式，比Find<package>.cmake方式更加通用。有关“模块”模式和“配置”模式的更多信息，可参考官方文档 https://cmake.org/cmake/help/v3.5/command/find_package.html 。

虽然Eigen3、BLAS和OpenMP声明为PUBLIC依赖项，但EIGEN_USE_BLAS编译定义声明为PRIVATE。可以在单独的库目标中汇集库依赖项，而不是直接链接可执行文件。使用PUBLIC/PRIVATE关键字，可以根据库目标的依赖关系调整相应标志和定义。

**更多信息**

CMake将在预定义的位置层次结构中查找配置模块。首先是CMAKE_PREFIX_PATH，<package>_DIR是接下来的搜索路径。因此，如果Eigen3安装在非标准位置，可以使用这两个选项来告诉CMake在哪里查找它:

* 通过将Eigen3的安装前缀传递给CMAKE_PREFIX_PATH:

.. code-block:: bash

  $ cmake -D CMAKE_PREFIX_PATH=<installation-prefix> ..

* 通过传递配置文件的位置作为Eigen3_DIR:

.. code-block:: bash

  $ cmake -D Eigen3_DIR=<installation-prefix>/share/eigen3/cmake ..

3.8 检测Boost库
--------------------------

.. NOTE::
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-08 中找到，包含一个C++的示例。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

Boost是一组C++通用库。这些库提供了许多功能，这些功能在现代C++项目中不可或缺，但是还不能通过C++标准使用这些功能。例如，Boost为元编程、处理可选参数和文件系统操作等提供了相应的组件。这些库中有许多特性后来被C++11、C++14和C++17标准所采用，但是对于保持与旧编译器兼容性的代码库来说，许多Boost组件仍然是首选。

本示例将向您展示如何检测和链接Boost库的一些组件。

**准备工作**

我们将编译的源码是Boost提供的文件系统库与文件系统交互的示例。这个库可以跨平台使用，并将操作系统和文件系统之间的差异抽象为一致的API。下面的代码(path-info.cpp)将接受一个路径作为参数，并将其组件的报告打印到屏幕上:

.. code-block:: c++

  #include <iostream>
  #include <boost/filesystem.hpp>
  using namespace std;
  using namespace boost::filesystem;
  const char *say_what(bool b) { return b ? "true" : "false"; }
  int main(int argc, char *argv[])
  {
    if (argc < 2)
    {
      cout
          << "Usage: path_info path-element [path-element...]\n"
            "Composes a path via operator/= from one or more path-element arguments\n"
            "Example: path_info foo/bar baz\n"
  #ifdef BOOST_POSIX_API
            " would report info about the composed path foo/bar/baz\n";
  #else // BOOST_WINDOWS_API
            " would report info about the composed path foo/bar\\baz\n";
  #endif
      return 1;
    }
    path p;
    for (; argc > 1; --argc, ++argv)
      p /= argv[1]; // compose path p from the command line arguments
    cout << "\ncomposed path:\n";
    cout << " operator<<()---------: " << p << "\n";
    cout << " make_preferred()-----: " << p.make_preferred() << "\n";
    cout << "\nelements:\n";
    for (auto element : p)
      cout << " " << element << '\n';
    cout << "\nobservers, native format:" << endl;
  #ifdef BOOST_POSIX_API
    cout << " native()-------------: " << p.native() << endl;
    cout << " c_str()--------------: " << p.c_str() << endl;
  #else // BOOST_WINDOWS_API
    wcout << L" native()-------------: " << p.native() << endl;
    wcout << L" c_str()--------------: " << p.c_str() << endl;
  #endif
    cout << " string()-------------: " << p.string() << endl;
    wcout << L" wstring()------------: " << p.wstring() << endl;
    cout << "\nobservers, generic format:\n";
    cout << " generic_string()-----: " << p.generic_string() << endl;
    wcout << L" generic_wstring()----: " << p.generic_wstring() << endl;
    cout << "\ndecomposition:\n";
    cout << " root_name()----------: " << p.root_name() << '\n';
    cout << " root_directory()-----: " << p.root_directory() << '\n';
    cout << " root_path()----------: " << p.root_path() << '\n';
    cout << " relative_path()------: " << p.relative_path() << '\n';
    cout << " parent_path()--------: " << p.parent_path() << '\n';
    cout << " filename()-----------: " << p.filename() << '\n';
    cout << " stem()---------------: " << p.stem() << '\n';
    cout << " extension()----------: " << p.extension() << '\n';
    cout << "\nquery:\n";
    cout << " empty()--------------: " << say_what(p.empty()) << '\n';
    cout << " is_absolute()--------: " << say_what(p.is_absolute()) << '\n';
    cout << " has_root_name()------: " << say_what(p.has_root_name()) << '\n';
    cout << " has_root_directory()-: " << say_what(p.has_root_directory()) << '\n';
    cout << " has_root_path()------: " << say_what(p.has_root_path()) << '\n';
    cout << " has_relative_path()--: " << say_what(p.has_relative_path()) << '\n';
    cout << " has_parent_path()----: " << say_what(p.has_parent_path()) << '\n';
    cout << " has_filename()-------: " << say_what(p.has_filename()) << '\n';
    cout << " has_stem()-----------: " << say_what(p.has_stem()) << '\n';
    cout << " has_extension()------: " << say_what(p.has_extension()) << '\n';
    return 0;
  }

**具体实施**

Boost由许多不同的库组成，这些库可以独立使用。CMake可将这个库集合，表示为组件的集合。FindBoost.cmake模块不仅可以搜索库集合的完整安装，还可以搜索集合中的特定组件及其依赖项(如果有的话)。我们将逐步建立相应的CMakeLists.txt:

1 首先，声明CMake最低版本、项目名称、语言，并使用C++11标准:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-08 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 然后，使用find_package搜索Boost。若需要对Boost强制性依赖，需要一个参数。这个例子中，只需要文件系统组件，所以将它作为参数传递给find_package:

.. code-block:: cmake

  find_package(Boost 1.54 REQUIRED COMPONENTS filesystem)

3 添加可执行目标，编译源文件:

.. code-block:: cmake

  add_executable(path-info path-info.cpp)

4 最后，将目标链接到Boost库组件。由于依赖项声明为PUBLIC，依赖于Boost的目标将自动获取依赖项:

.. code-block:: cmake

  target_link_libraries(path-info
    PUBLIC
        Boost::filesystem
      )

**工作原理**

FindBoost.cmake是本示例中所使用的CMake模块，其会在标准系统安装目录中找到Boost库。由于我们链接的是Boost::filesystem，CMake将自动设置包含目录并调整编译和链接标志。如果Boost库安装在非标准位置，可以在配置时使用BOOST_ROOT变量传递Boost安装的根目录，以便让CMake搜索非标准路径:

.. code-block:: bash

  $ cmake -D BOOST_ROOT=/custom/boost

或者，可以同时传递包含头文件的BOOST_INCLUDEDIR变量和库目录的BOOST_LIBRARYDIR变量:

.. code-block:: bash

  $ cmake -D BOOST_INCLUDEDIR=/custom/boost/include -DBOOST_LIBRARYDIR=/custom/boost/lib

3.9 检测外部库:Ⅰ. 使用pkg-config
---------------------------------------

.. NOTE::
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-09 中找到，包含一个C的示例。该示例在CMake 3.6版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-09 中也有一个适用于CMake 3.5的示例。

目前为止，我们已经讨论了两种检测外部依赖关系的方法:

使用CMake自带的find-module，但并不是所有的包在CMake的find模块都找得到。
使用<package>Config.cmake, <package>ConfigVersion.cmake和<package>Targets.cmake，这些文件由软件包供应商提供，并与软件包一起安装在标准位置的cmake文件夹下。
如果某个依赖项既不提供查找模块，也不提供供应商打包的CMake文件，该怎么办?在这种情况下，我们只有两个选择:

依赖pkg-config程序，来找到系统上的包。这依赖于包供应商在.pc配置文件中，其中有关于发行包的元数据。
为依赖项编写自己的find-package模块。
本示例中，将展示如何利用CMake中的pkg-config来定位ZeroMQ消息库。下一个示例中，将编写一个find模块，展示如何为ZeroMQ编写属于自己find模块。

**准备工作**

我们构建的代码来自ZeroMQ手册 http://zguide.zeromq.org/page:all 的示例。由两个源文件hwserver.c和hwclient.c组成，这两个源文件将构建为两个独立的可执行文件。执行时，它们将打印“Hello, World”。

**具体实施**

这是一个C项目，我们将使用C99标准，逐步构建CMakeLists.txt文件:

1 声明一个C项目，并要求符合C99标准:

.. code-block:: bash

  cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
  project(recipe-09 LANGUAGES C)
  set(CMAKE_C_STANDARD 99)
  set(CMAKE_C_EXTENSIONS OFF)
  set(CMAKE_C_STANDARD_REQUIRED ON)

2 使用CMake附带的find-module，查找pkg-config。这里在find_package中传递了QUIET参数。只有在没有找到pkg-config时，CMake才会报错:

.. code-block:: cmake

  find_package(PkgConfig REQUIRED QUIET)

3 找到pkg-config时，我们将使用pkg_search_module函数，以搜索任何附带包配置.pc文件的库或程序。该示例中，我们查找ZeroMQ库:

.. code-block:: bash

  pkg_search_module(
    ZeroMQ
    REQUIRED
        libzeromq libzmq lib0mq
    IMPORTED_TARGET
    )

4 如果找到ZeroMQ库，则打印状态消息:

.. code-block:: cmake

  if(TARGET PkgConfig::ZeroMQ)
      message(STATUS "Found ZeroMQ")
  endif()

5 然后，添加两个可执行目标，并链接到ZeroMQ。这将自动设置包括目录和链接库:

  add_executable(hwserver hwserver.c)
  target_link_libraries(hwserver PkgConfig::ZeroMQ)
  add_executable(hwclient hwclient.c)
  target_link_libraries(hwclient PkgConfig::ZeroMQ)

6 现在，我们可以配置和构建示例:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .

7 在终端中，启动服务器，启动时会输出类似于本例的消息:
'
.. code-block:: bash

  urrent 0MQ version is 4.2.2

8 然后，在另一个终端启动客户端，它将打印如下内容:

.. code-block:: bash

  Connecting to hello world server…
  Sending Hello 0…
  Received World 0
  Sending Hello 1…
  Received World 1
  Sending Hello 2…
  ...

**工作**

当找到pkg-config时, CMake需要提供两个函数，来封装这个程序提供的功能:

* pkg_check_modules，查找传递列表中的所有模块(库和/或程序)
* pkg_search_module，要在传递的列表中找到第一个工作模块

与find_package一样，这些函数接受REQUIRED和QUIET参数。更详细地说，我们对pkg_search_module的调用如下:

.. code-block:: cmake

  pkg_search_module(
    ZeroMQ
    REQUIRED
        libzeromq libzmq lib0mq
    IMPORTED_TARGET
    )

这里，第一个参数是前缀，它将用于命名存储搜索ZeroMQ库结果的目标：PkgConfig::ZeroMQ。注意，我们需要为系统上的库名传递不同的选项：libzeromq、libzmq和lib0mq。这是因为不同的操作系统和包管理器，可为同一个包选择不同的名称。

.. NOTE::
  
  pkg_check_modules和pkg_search_module函数添加了IMPORTED_TARGET选项，并在CMake 3.6中定义导入目标的功能。3.6之前的版本，只定义了变量ZeroMQ_INCLUDE_DIRS(用于include目录)和ZeroMQ_LIBRARIES(用于链接库)，供后续使用。

3.10 检测外部库:Ⅱ. 自定义find模块
---------------------------------------

.. NOTE::
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-03/recipe-10 中找到，包含一个C的示例。该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

此示例补充了上一节的示例，我们将展示如何编写一个find模块来定位系统上的ZeroMQ消息库，以便能够在非Unix操作系统上检测该库。我们重用服务器-客户端示例代码。

*8如何实施**

这是一个C项目，使用C99标准，并逐步构建CMakeLists.txt文件:

1 声明一个C项目，并要求符合C99标准:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-10 LANGUAGES C)
  set(CMAKE_C_STANDARD 99)
  set(CMAKE_C_EXTENSIONS OFF)
  set(CMAKE_C_STANDARD_REQUIRED ON)

2 将当前源目录CMAKE_CURRENT_SOURCE_DIR，添加到CMake将查找模块的路径列表CMAKE_MODULE_PATH中。这样CMake就可以找到，我们自定义的FindZeroMQ.cmake模块:

.. code-block:: cmake

  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

3 现在FindZeroMQ.cmake模块是可用的，可以通过这个模块来搜索项目所需的依赖项。由于我们没有使用QUIET选项来查找find_package，所以当找到库时，状态消息将自动打印:

.. code-block:: cmake

  find_package(ZeroMQ REQUIRED)

4 我们继续添加hwserver可执行目标。头文件包含目录和链接库是使用find_package命令成功后，使用ZeroMQ_INCLUDE_DIRS和ZeroMQ_LIBRARIES变量进行指定的:

.. code-block:: cmake

  add_executable(hwserver hwserver.c)
  target_include_directories(hwserver
    PRIVATE
        ${ZeroMQ_INCLUDE_DIRS}
    )
  target_link_libraries(hwserver
    PRIVATE
        ${ZeroMQ_LIBRARIES}
    )

5 最后，我们对hwclient可执行目标执行相同的操作:

.. code-block:: cmake

  add_executable(hwclient hwclient.c)
  target_include_directories(hwclient
    PRIVATE
        ${ZeroMQ_INCLUDE_DIRS}
    )
  target_link_libraries(hwclient
    PRIVATE
        ${ZeroMQ_LIBRARIES}
    )

此示例的主CMakeLists.txt在使用FindZeroMQ.cmake时，与前一个示例中使用的CMakeLists.txt不同。这个模块使用find_path和find_library CMake内置命令，搜索ZeroMQ头文件和库，并使用find_package_handle_standard_args设置相关变量，就像我们在第3节中做的那样。

1 FindZeroMQ.cmake中，检查了ZeroMQ_ROOT变量是否设置。此变量可用于ZeroMQ库的检测，并引导到自定义安装目录。用户可能设置了ZeroMQ_ROOT作为环境变量，我们也会进行检查了:

.. code-block:: cmake

  if(NOT ZeroMQ_ROOT)
      set(ZeroMQ_ROOT "$ENV{ZeroMQ_ROOT}")
  endif()

2 然后，搜索系统上zmq.h头文件的位置。这是基于_ZeroMQ_ROOT变量和find_path命令进行的:

.. code-block:: cmake

  if(NOT ZeroMQ_ROOT)
      find_path(_ZeroMQ_ROOT NAMES include/zmq.h)
  else()
      set(_ZeroMQ_ROOT "${ZeroMQ_ROOT}")
  endif()
  find_path(ZeroMQ_INCLUDE_DIRS NAMES zmq.h HINTS ${_ZeroMQ_ROOT}/include)

3 如果成功找到头文件，则将ZeroMQ_INCLUDE_DIRS设置为其位置。我们继续通过使用字符串操作和正则表达式，寻找相应版本的ZeroMQ库:

.. code-block:: cmake

  set(_ZeroMQ_H ${ZeroMQ_INCLUDE_DIRS}/zmq.h)
  function(_zmqver_EXTRACT _ZeroMQ_VER_COMPONENT _ZeroMQ_VER_OUTPUT)
  set(CMAKE_MATCH_1 "0")
  set(_ZeroMQ_expr "^[ \\t]*#define[ \\t]+${_ZeroMQ_VER_COMPONENT}[ \\t]+([0-9]+)$")
  file(STRINGS "${_ZeroMQ_H}" _ZeroMQ_ver REGEX "${_ZeroMQ_expr}")
  string(REGEX MATCH "${_ZeroMQ_expr}" ZeroMQ_ver "${_ZeroMQ_ver}")
  set(${_ZeroMQ_VER_OUTPUT} "${CMAKE_MATCH_1}" PARENT_SCOPE)
  endfunction()
  _zmqver_EXTRACT("ZMQ_VERSION_MAJOR" ZeroMQ_VERSION_MAJOR)
  _zmqver_EXTRACT("ZMQ_VERSION_MINOR" ZeroMQ_VERSION_MINOR)
  _zmqver_EXTRACT("ZMQ_VERSION_PATCH" ZeroMQ_VERSION_PATCH)

4 然后，为find_package_handle_standard_args准备ZeroMQ_VERSION变量:

.. code-block:: cmake

  if(ZeroMQ_FIND_VERSION_COUNT GREATER 2)
      set(ZeroMQ_VERSION "${ZeroMQ_VERSION_MAJOR}.${ZeroMQ_VERSION_MINOR}.${ZeroMQ_VERSION_PATCH}")
  else()
      set(ZeroMQ_VERSION "${ZeroMQ_VERSION_MAJOR}.${ZeroMQ_VERSION_MINOR}")
  endif()

5 使用find_library命令搜索ZeroMQ库。因为库的命名有所不同，这里我们需要区分Unix的平台和Windows平台:

.. code-block:: cmake

  if(NOT ${CMAKE_C_PLATFORM_ID} STREQUAL "Windows")
    find_library(ZeroMQ_LIBRARIES
      NAMES
          zmq
      HINTS
        ${_ZeroMQ_ROOT}/lib
        ${_ZeroMQ_ROOT}/lib/x86_64-linux-gnu
      )
  else()
    find_library(ZeroMQ_LIBRARIES
      NAMES
          libzmq
        "libzmq-mt-${ZeroMQ_VERSION_MAJOR}_${ZeroMQ_VERSION_MINOR}_${ZeroMQ_VERSION_PATCH}"
        "libzmq-${CMAKE_VS_PLATFORM_TOOLSET}-mt-${ZeroMQ_VERSION_MAJOR}_${ZeroMQ_VERSION_MINOR}_${ZeroMQ_VERSION_PATCH}"
        libzmq_d
        "libzmq-mt-gd-${ZeroMQ_VERSION_MAJOR}_${ZeroMQ_VERSION_MINOR}_${ZeroMQ_VERSION_PATCH}"
        "libzmq-${CMAKE_VS_PLATFORM_TOOLSET}-mt-gd-${ZeroMQ_VERSION_MAJOR}_${ZeroMQ_VERSION_MINOR}_${ZeroMQ_VERSION_PATCH}"
      HINTS
          ${_ZeroMQ_ROOT}/lib
      )
  endif()

6 最后，包含了标准FindPackageHandleStandardArgs.cmake，并调用相应的CMake命令。如果找到所有需要的变量，并且版本匹配，则将ZeroMQ_FOUND变量设置为TRUE:

.. code-block:: cmake

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(ZeroMQ
    FOUND_VAR
        ZeroMQ_FOUND
    REQUIRED_VARS
    ZeroMQ_INCLUDE_DIRS
    ZeroMQ_LIBRARIES
    VERSION_VAR
    ZeroMQ_VERSION
    )

.. NOTE:: 
  
  刚才描述的FindZeroMQ.cmake模块已经在 https://github.com/zeromq/azmq/blob/master/config/FindZeroMQ.cmake 上进行了修改。

**工作原理**

find-module通常遵循特定的模式:

1 检查用户是否为所需的包提供了自定义位置。

2 使用find_家族中的命令搜索所需包的必需组件，即头文件、库、可执行程序等等。我们使用find_path查找头文件的完整路径，并使用find_library查找库。CMake还提供find_file、find_program和find_package。这些命令的签名如下:

.. code-block:: cmake

  find_path(<VAR> NAMES name PATHS paths)

3 如果搜索成功，<VAR>将保存搜索结果；如果搜索失败，则会设置为<VAR>-NOTFOUND。NAMES和PATHS分别是CMake应该查找的文件的名称和搜索应该指向的路径。

4 初步搜索的结果中，可以提取版本号。示例中，ZeroMQ头文件包含库版本，可以使用字符串操作和正则表达式提取库版本信息。

5 最后，调用find_package_handle_standard_args命令。处理find_package命令的REQUIRED、QUIET和版本参数，并设置ZeroMQ_FOUND变量。

.. NOTE::

  任何CMake命令的完整文档都可以从命令行获得。例如，cmake --help-command find_file将输出find_file命令的手册页。对于CMake标准模块的手册，可以在CLI使用--help-module看到。例如，cmake --help-module FindPackageHandleStandardArgs将输出FindPackageHandleStandardArgs.cmake的手册页面。

**更多信息**

总而言之，有四种方式可用于找到依赖包:

1 使用由包供应商提供CMake文件<package>Config.cmake ，<package>ConfigVersion.cmake和<package>Targets.cmake，通常会在包的标准安装位置查找。

2 无论是由CMake还是第三方提供的模块，为所需包使用find-module。

3 使用pkg-config，如本节的示例所示。

4 如果这些都不可行，那么编写自己的find模块。

这四种可选方案按相关性进行了排序，每种方法也都有其挑战。

目前，并不是所有的包供应商都提供CMake的Find文件，不过正变得越来越普遍。因为导出CMake目标，使得第三方代码很容易使用它所依赖的库和/或程序附加的依赖。

从一开始，Find-module就一直是CMake中定位依赖的主流手段。但是，它们中的大多数仍然依赖于设置依赖项使用的变量，比如Boost_INCLUDE_DIRS、PYTHON_INTERPRETER等等。这种方式很难在第三方发布自己的包时，确保依赖关系被满足。

使用pkg-config的方法可以很好地进行适配，因为它已经成为Unix系统的标准。然而，也由于这个原因，它不是一个完全跨平台的方法。此外，如CMake文档所述，在某些情况下，用户可能会意外地覆盖检测包，并导致pkg-config提供不正确的信息。

最后的方法是编写自己的查找模块脚本，就像本示例中那样。这是可行的，并且依赖于FindPackageHandleStandardArgs.cmake。然而，编写一个全面的查找模块脚本绝非易事；有需要考虑很多可能性，我们在Unix和Windows平台上，为查找ZeroMQ库文件演示了一个例子。

所有软件开发人员都非常清楚这些问题和困难，正如CMake邮件列表上讨论所示: https://cmake.org/pipermail/cmake/2018-May/067556.html 。pkg-config在Unix包开发人员中是可以接受的，但是它不能很容易地移植到非Unix平台。CMake配置文件功能强大，但并非所有软件开发人员都熟悉CMake语法。公共包规范项目是统一用于包查找的pkg-config和CMake配置文件方法的最新尝试。您可以在项目的网站上找到更多信息: https://mwoehlke.github.io/cps/

在第10章中将讨论，如何使用前面讨论中概述的第一种方法，使第三方应用程序，找到自己的包：为项目提供自己的CMake查找文件。
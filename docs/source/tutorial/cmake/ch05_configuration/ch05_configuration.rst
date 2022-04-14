.. highlight:: c++

.. default-domain:: cpp

==========================
第5章 配置时和构建时的操作
==========================

5.1 使用平台无关的文件操作
---------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-01 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

有些项目构建时，可能需要与平台的文件系统进行交互。也就是检查文件是否存在、创建新文件来存储临时信息、创建或提取打包文件等等。
使用CMake不仅能够在不同的平台上生成构建系统，还能够在不复杂的逻辑情况下，进行文件操作，从而独立于操作系统。本示例将展示，如何以可移植的方式下载库文件。

**准备工作**

我们将展示如何提取Eigen库文件，并使用提取的源文件编译我们的项目。这个示例中，将重用第3章第7节的线性代数例子linear-algebra.cpp，
用来检测外部库和程序、检测特征库。这里，假设已经包含Eigen库文件，已在项目构建前下载。

**具体实施**

项目需要解压缩Eigen打包文件，并相应地为目标设置包含目录:

1 首先，使能C++11项目:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 我们将自定义目标添加到构建系统中，自定义目标将提取构建目录中的库文件:

.. code-block:: cmake

  add_custom_target(unpack-eigen
    ALL
    COMMAND
        ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/eigen-eigen-5a0156e40feb.tar.gz
    COMMAND
        ${CMAKE_COMMAND} -E rename eigen-eigen-5a0156e40feb eigen-3.3.4
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT
        "Unpacking Eigen3 in ${CMAKE_CURRENT_BINARY_DIR}/eigen-3.3.4"
    )

3 为源文件添加了一个可执行目标:

.. code-block:: cmake

  add_executable(linear-algebra linear-algebra.cpp)

4 由于源文件的编译依赖于Eigen头文件，需要显式地指定可执行目标对自定义目标的依赖关系:

.. code-block:: cmake

  add_dependencies(linear-algebra unpack-eigen)

5 最后，指定包含哪些目录:

.. code-block:: cmake

  target_include_directories(linear-algebra
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/eigen-3.3.4
    )

**工作原理**

细看add_custom_target这个命令：

.. code-block:: cmake

  add_custom_target(unpack-eigen
    ALL
    COMMAND
        ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/eigen-eigen-5a0156e40feb.tar.gz
    COMMAND
        ${CMAKE_COMMAND} -E rename eigen-eigen-5a0156e40feb eigen-3.3.4
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT
        "Unpacking Eigen3 in ${CMAKE_CURRENT_BINARY_DIR}/eigen-3.3.4"
    )

构建系统中引入了一个名为unpack-eigen的目标。因为我们传递了ALL参数，目标将始终被执行。COMMAND参数指定要执行哪些命令。
本例中，我们希望提取存档并将提取的目录重命名为egan -3.3.4，通过以下两个命令实现:

.. code-block:: bash

    ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/eigen-eigen-5a0156e40feb.tar.gz
    ${CMAKE_COMMAND} -E rename eigen-eigen-5a0156e40feb eigen-3.3.4

注意，使用-E标志调用CMake命令本身来执行实际的工作。对于许多常见操作，CMake实现了一个对所有操作系统都通用的接口，这使得构建系统独立于特定的平台。add_custom_target命令中的下一个参数是工作目录。我们的示例中，它对应于构建目录：CMAKE_CURRENT_BINARY_DIR。最后一个参数COMMENT，用于指定CMake在执行自定义目标时输出什么样的消息。

**更多信息**

构建过程中必须执行一系列没有输出的命令时，可以使用add_custom_target命令。正如我们在本示例中所示，可以将自定义目标指定为项目中其他目标的依赖项。
此外，自定义目标还可以依赖于其他目标。

使用-E标志可以以与操作系统无关的方式，运行许多公共操作。运行cmake -E或cmake -E help可以获得特定操作系统的完整列表。
例如，这是Linux系统上命令的摘要:

.. code-block:: bash

  Usage: cmake -E <command> [arguments...]
  Available commands:
    capabilities              - Report capabilities built into cmake in JSON format
    chdir dir cmd [args...]   - run command in a given directory
    compare_files file1 file2 - check if file1 is same as file2
    copy <file>... destination  - copy files to destination (either file or directory)
    copy_directory <dir>... destination   - copy content of <dir>... directories to 'destination' directory
    copy_if_different <file>... destination  - copy files if it has changed
    echo [<string>...]        - displays arguments as text
    echo_append [<string>...] - displays arguments as text but no new line
    env [--unset=NAME]... [NAME=VALUE]... COMMAND [ARG]...
                              - run command in a modified environment
    environment               - display the current environment
    make_directory <dir>...   - create parent and <dir> directories
    md5sum <file>...          - create MD5 checksum of files
    sha1sum <file>...         - create SHA1 checksum of files
    sha224sum <file>...       - create SHA224 checksum of files
    sha256sum <file>...       - create SHA256 checksum of files
    sha384sum <file>...       - create SHA384 checksum of files
    sha512sum <file>...       - create SHA512 checksum of files
    remove [-f] <file>...     - remove the file(s), use -f to force it
    remove_directory dir      - remove a directory and its contents
    rename oldname newname    - rename a file or directory (on one volume)
    server                    - start cmake in server mode
    sleep <number>...         - sleep for given number of seconds
    tar [cxt][vf][zjJ] file.tar [file/dir1 file/dir2 ...]
                              - create or extract a tar or zip archive
    time command [args...]    - run command and display elapsed time
    touch file                - touch a file.
    touch_nocreate file       - touch a file but do not create it.
  Available on UNIX only:
    create_symlink old new    - create a symbolic link new -> old

5.2 配置时运行自定义命令
---------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-05/recipe-02 中找到。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

运行CMake生成构建系统，从而指定原生构建工具必须执行哪些命令，以及按照什么顺序执行。我们已经了解了CMake如何在配置时运行许多子任务，
以便找到工作的编译器和必要的依赖项。本示例中，我们将讨论如何使用execute_process命令在配置时运行定制化命令。

**具体实施**

第3章第3节中，我们已经展示了execute_process查找Python模块NumPy时的用法。本例中，我们将使用execute_process命令来确定，
是否存在特定的Python模块(本例中为Python CFFI)，如果存在，我们在进行版本确定:

1 对于这个简单的例子，不需要语言支持:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-02 LANGUAGES NONE)

2 我们要求Python解释器执行一个简短的代码片段，因此，需要使用find_package来查找解释器：
.. code-block:: cmake

  find_package(PythonInterp REQUIRED)

3 然后，调用execute_process来运行一个简短的Python代码段；下一节中，我们将更详细地讨论这个命令:

.. code-block:: cmake

  # this is set as variable to prepare
  # for abstraction using loops or functions
  set(_module_name "cffi")
  execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} "-c" "import ${_module_name}; print(${_module_name}.__version__)"
    OUTPUT_VARIABLE _stdout
    ERROR_VARIABLE _stderr
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
    )

4 然后，打印结果：

.. code-block:: cmake

  if(_stderr MATCHES "ModuleNotFoundError")
      message(STATUS "Module ${_module_name} not found")
  else()
      message(STATUS "Found module ${_module_name} v${_stdout}")
  endif()

5 下面是一个配置示例(假设Python CFFI包安装在相应的Python环境中):

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  -- Found PythonInterp: /home/user/cmake-cookbook/chapter-05/recipe-02/example/venv/bin/python (found version "3.6.5")
  -- Found module cffi v1.11.5

**工作原理**

execute_process命令将从当前正在执行的CMake进程中派生一个或多个子进程，从而提供了在配置项目时运行任意命令的方法。
可以在一次调用execute_process时执行多个命令。但请注意，每个命令的输出将通过管道传输到下一个命令中。该命令接受多个参数:

* WORKING_DIRECTORY，指定应该在哪个目录中执行命令。
* RESULT_VARIABLE将包含进程运行的结果。这要么是一个整数，表示执行成功，要么是一个带有错误条件的字符串。
* OUTPUT_VARIABLE和ERROR_VARIABLE将包含执行命令的标准输出和标准错误。由于命令的输出是通过管道传输的，因此只有最后一个命令的标准输出才会保存到OUTPUT_VARIABLE中。
* INPUT_FILE指定标准输入重定向的文件名
* OUTPUT_FILE指定标准输出重定向的文件名
* ERROR_FILE指定标准错误输出重定向的文件名
* 设置OUTPUT_QUIET和ERROR_QUIET后，CMake将静默地忽略标准输出和标准错误。
* 设置OUTPUT_STRIP_TRAILING_WHITESPACE，可以删除运行命令的标准输出中的任何尾随空格
* 设置ERROR_STRIP_TRAILING_WHITESPACE，可以删除运行命令的错误输出中的任何尾随空格。

有了这些了解这些参数，回到我们的例子当中:

.. code-block:: cmake

  set(_module_name "cffi")
  execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} "-c" "import ${_module_name}; print(${_module_name}.__version__)"
    OUTPUT_VARIABLE _stdout
    ERROR_VARIABLE _stderr
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
    )
  if(_stderr MATCHES "ModuleNotFoundError")
      message(STATUS "Module ${_module_name} not found")
  else()
    message(STATUS "Found module ${_module_name} v${_stdout}")
  endif()

该命令检查python -c "import cffi; print(cffi.__version__)"的输出。如果没有找到模块，_stderr将包含ModuleNotFoundError，
我们将在if语句中对其进行检查。本例中，我们将打印Module cffi not found。如果导入成功，Python代码将打印模块的版本，
该模块通过管道输入_stdout，这样就可以打印如下内容:

.. code-block:: cmake

  message(STATUS "Found module ${_module_name} v${_stdout}")

5.3 构建时运行自定义命令:Ⅰ. 使用add_custom_command
---------------------------------------------------

.. NOTE:: 
  
  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-03 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

项目的构建目标取决于命令的结果，这些命令只能在构建系统生成完成后的构建执行。CMake提供了三个选项来在构建时执行自定义命令:

1 使用add_custom_command编译目标，生成输出文件。
2 add_custom_target的执行没有输出。
3 构建目标前后，add_custom_command的执行可以没有输出。

这三个选项强制执行特定的语义，并且不可互换。接下来的三个示例将演示具体的用法。

**准备工作**

我们将重用第3章第4节中的C++示例，以说明如何使用add_custom_command的第一个选项。代码示例中，我们了解了现有的BLAS和LAPACK库，
并编译了一个很小的C++包装器库，以调用线性代数的Fortran实现。

我们将把代码分成两部分。linear-algebra.cpp的源文件与第3章、第4章没有区别，并且将包含线性代数包装器库的头文件和针对编译库的链接。
源代码将打包到一个压缩的tar存档文件中，该存档文件随示例项目一起提供。存档文件将在构建时提取，并在可执行文件生成之前，编译线性代数的包装器库。

**具体实施**

CMakeLists.txt必须包含一个自定义命令，来提取线性代数包装器库的源代码：

1 从CMake最低版本、项目名称和支持语言的定义开始:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-03 LANGUAGES CXX Fortran)

2 选择C++11标准:

.. code-block:: cmake

  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

3 然后，在系统上查找BLAS和LAPACK库:

.. code-block:: cmake

  find_package(BLAS REQUIRED)
  find_package(LAPACK REQUIRED)

4 声明一个变量wrap_BLAS_LAPACK_sources来保存wrap_BLAS_LAPACK.tar.gz压缩包文件的名称:

.. code-block:: cmake

  set(wrap_BLAS_LAPACK_sources
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.cpp
    )

5 声明自定义命令来提取wrap_BLAS_LAPACK.tar.gz压缩包，并更新提取文件的时间戳。注意这个wrap_BLAS_LAPACK_sources变量的预期输出:

.. code-block:: cmake

  add_custom_command(
    OUTPUT
        ${wrap_BLAS_LAPACK_sources}
    COMMAND
        ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/wrap_BLAS_LAPACK.tar.gz
    COMMAND
        ${CMAKE_COMMAND} -E touch ${wrap_BLAS_LAPACK_sources}
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/wrap_BLAS_LAPACK.tar.gz
    COMMENT
        "Unpacking C++ wrappers for BLAS/LAPACK"
    VERBATIM
    )

6 接下来，添加一个库目标，源文件是新解压出来的:

.. code-block:: cmake

  add_library(math "")
  target_sources(math
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.cpp
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.hpp
        ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.hpp
    )
  target_include_directories(math
    INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK
    )
  target_link_libraries(math
    PUBLIC
        ${LAPACK_LIBRARIES}
    )

7 最后，添加linear-algebra可执行目标。可执行目标链接到库:

.. code-block:: cmake

  add_executable(linear-algebra linear-algebra.cpp)
  target_link_libraries(linear-algebra
    PRIVATE
        math
    )

我们配置、构建和执行示例:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./linear-algebra 1000
  C_DSCAL done
  C_DGESV done
  info is 0
  check is 4.35597e-10

**工作原理**

让我们来了解一下add_custom_command的使用:

.. code-block:: cmake

  add_custom_command(
    OUTPUT
        ${wrap_BLAS_LAPACK_sources}
    COMMAND
        ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/wrap_BLAS_LAPACK.tar.gz
    COMMAND
        ${CMAKE_COMMAND} -E touch ${wrap_BLAS_LAPACK_sources}
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/wrap_BLAS_LAPACK.tar.gz
    COMMENT
        "Unpacking C++ wrappers for BLAS/LAPACK"
    VERBATIM
    )

add_custom_command向目标添加规则，并通过执行命令生成输出。add_custom_command中声明的任何目标，即在相同的CMakeLists.txt中声明的任何目标，
使用输出的任何文件作为源文件的目标，在构建时会有规则生成这些文件。因此，源文件生成在构建时，目标和自定义命令在构建系统生成时，将自动处理依赖关系。

我们的例子中，输出是压缩tar包，其中包含有源文件。要检测和使用这些文件，必须在构建时提取打包文件。通过使用带有-E标志的CMake命令，以实现平台独立性。
下一个命令会更新提取文件的时间戳。这样做是为了确保没有处理陈旧文件。WORKING_DIRECTORY可以指定在何处执行命令。
示例中，CMAKE_CURRENT_BINARY_DIR是当前正在处理的构建目录。DEPENDS参数列出了自定义命令的依赖项。例子中，压缩的tar是一个依赖项。
CMake使用COMMENT字段在构建时打印状态消息。最后，VERBATIM告诉CMake为生成器和平台生成正确的命令，从而确保完全独立。

我们来仔细看看这用使用方式和打包库的创建：

.. code-block:: cmake

  add_library(math "")
  target_sources(math
    PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.cpp
      ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.cpp
    PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.hpp
      ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.hpp
    )
  target_include_directories(math
    INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK
    )
  target_link_libraries(math
    PUBLIC
        ${LAPACK_LIBRARIES}
    )

我们声明一个没有源的库目标，是因为后续使用target_sources填充目标的源。这里实现了一个非常重要的目标，即让依赖于此目标的目标，
了解需要哪些目录和头文件，以便成功地使用库。C++源文件的目标是PRIVATE，因此只用于构建库。因为目标及其依赖项都需要使用它们来成功编译，
所以头文件是PUBLIC。包含目录使用target_include_categories指定，其中wrap_BLAS_LAPACK声明为INTERFACE，因为只有依赖于math目标的目标需要它。

add_custom_command有两个限制:

* 只有在相同的CMakeLists.txt中，指定了所有依赖于其输出的目标时才有效。
* 对于不同的独立目标，使用add_custom_command的输出可以重新执行定制命令。这可能会导致冲突，应该避免这种情况的发生。

第二个限制，可以使用add_dependencies来避免。不过，规避这两个限制的正确方法是使用add_custom_target命令，我们将在下一节的示例中详细介绍。

5.4 构建时运行自定义命令:Ⅱ. 使用add_custom_target
---------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-04 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

我们在前面的示例，讨论了add_custom_command有一些限制，可以通过add_custom_target绕过这些限制。这个CMake命令将引入新的目标，与add_custom_command相反，这些目标依次执行不返回输出。可以将add_custom_target和add_custom_command结合使用。使用这种方法，可以与其依赖项所在目录不同的目录指定自定义目标，CMake基础设施对项目设计模块化非常有用。

**准备工作**

我们将重用前一节示例，对源码进行简单的修改。特别是，将把压缩后的tar打包文件放在名为deps的子目录中，而不是存储在主目录中。
这个子目录包含它自己的CMakeLists.txt，将由主CMakeLists.txt调用。

**具体实施**

我们将从主CMakeLists.txt开始，然后讨论deps/CMakeLists.txt:

1 声明启用C++11：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-04 LANGUAGES CXX Fortran)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 现在，继续讨论deps/CMakeLists.txt。这通过add_subdirectory命令实现:

.. code-block:: bash

  add_subdirectory(deps)
  deps/CMakeLists.txt中，我们首先定位必要的库(BLAS和LAPACK):

  find_package(BLAS REQUIRED)
  find_package(LAPACK REQUIRED)

4 然后，我们将tar包的内容汇集到一个变量MATH_SRCS中:

.. code-block:: cmake

  set(MATH_SRCS
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.hpp
    )

5 列出要打包的源之后，定义一个目标和一个命令。这个组合用于提取${CMAKE_CURRENT_BINARY_DIR}中的包。但是，这里我们在一个不同的范围内，
  引用deps/CMakeLists.txt，因此tar包将存放在到主项目构建目录下的deps子目录中:

.. code-block:: cmake

  add_custom_target(BLAS_LAPACK_wrappers
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS
        ${MATH_SRCS}
    COMMENT
        "Intermediate BLAS_LAPACK_wrappers target"
    VERBATIM
    )
  add_custom_command(
    OUTPUT
        ${MATH_SRCS}
    COMMAND
        ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/wrap_BLAS_LAPACK.tar.gz
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/wrap_BLAS_LAPACK.tar.gz
    COMMENT
        "Unpacking C++ wrappers for BLAS/LAPACK"
    )

6 添加数学库作为目标，并指定相应的源，包括目录和链接库:

.. code-block:: cmake

  add_library(math "")
  target_sources(math
    PRIVATE
        ${MATH_SRCS}
    )
  target_include_directories(math
    INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK
    )
  # BLAS_LIBRARIES are included in LAPACK_LIBRARIES
  target_link_libraries(math
    PUBLIC
        ${LAPACK_LIBRARIES}
    )

7 执行完deps/CMakeLists.txt中的命令，返回到父范围，定义可执行目标，并将其链接到另一个目录的数学库:

.. code-block:: cmake

  add_executable(linear-algebra linear-algebra.cpp)
  target_link_libraries(linear-algebra
    PRIVATE
        math
    )

**工作原理**

用户可以使用add_custom_target，在目标中执行定制命令。这与我们前面讨论的add_custom_command略有不同。add_custom_target添加的目标没有输出，
因此总会执行。因此，可以在子目录中引入自定义目标，并且仍然能够在主CMakeLists.txt中引用它。

本例中，使用add_custom_target和add_custom_command提取了源文件的包。这些源文件稍后用于编译另一个库，我们设法在另一个(父)目录范围内链接这个库。
构建CMakeLists.txt文件的过程中，tar包是在deps下，deps是项目构建目录下的一个子目录。这是因为在CMake中，构建树的结构与源树的层次结构相同。

这个示例中有一个值得注意的细节，就是我们把数学库的源标记为PRIVATE:

.. code-block:: cmake

  set(MATH_SRCS
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxBLAS.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/wrap_BLAS_LAPACK/CxxLAPACK.hpp
    )
  # ...
  add_library(math "")
  target_sources(math
    PRIVATE
        ${MATH_SRCS}
    )
  # ...

虽然这些源代码是PRIVATE，但我们在父范围内编译了linear-algebra.cpp，并且这个源代码包括CxxBLAS.hpp和CxxLAPACK.hpp。
为什么这里使用PRIVATE，以及如何编译linear-algebra.cpp，并构建可执行文件呢？如果将头文件标记为PUBLIC, CMake就会在创建时停止，并出现一个错误，
“无法找到源文件”，因为要生成(提取)还不存在于文件树中的源文件。

这是一个已知的限制(参见https://gitlab.kitware.com/cmake/cmake/issues/1633 ，
以及相关的博客文章:https://samthursfield.wordpress.com/2015/11/21/cmake-depende-ncies-targets-and-files-and-custom-commands )。
我们通过声明源代码为PRIVATE来解决这个限制。这样CMake时，没有获得对不存在源文件的依赖。但是，CMake内置的C/C++文件依赖关系扫描器在构建时获取它们，
并编译和链接源代码。

5.5 构建时为特定目标运行自定义命令
---------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-05 中找到，其中包含一个Fortran例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

本节示例将展示，如何使用add_custom_command的第二个参数，来执行没有输出的自定义操作，这对于构建或链接特定目标之前或之后执行某些操作非常有用。
由于自定义命令仅在必须构建目标本身时才执行，因此我们实现了对其执行的目标级控制。我们将通过一个示例来演示，在构建目标之前打印目标的链接，然后在编译后，
立即测量编译后，可执行文件的静态分配大小。

**准备工作**

本示例中，我们将使用Fortran代码(example.f90):

.. code-block:: bash

  program example
    implicit none
    real(8) :: array(20000000)
    real(8) :: r
    integer :: i
    do i = 1, size(array)
      call random_number(r)
      array(i) = r
    end do
    print *, sum(array)
  end program

虽然我们选择了Fortran，但Fortran代码的对于后面的讨论并不重要，因为有很多遗留的Fortran代码，存在静态分配大小的问题。

这段代码中，我们定义了一个包含20,000,000双精度浮点数的数组，这个数组占用160MB的内存。在这里，我们并不是推荐这样的编程实践。一般来说，
这些内存的分配和代码中是否使用这段内存无关。一个更好的方法是只在需要时动态分配数组，随后立即释放。

示例代码用随机数填充数组，并计算它们的和——这样是为了确保数组确实被使用，并且编译器不会优化分配。我们将使用Python脚本(static-size.py)
来统计二进制文件静态分配的大小，该脚本用size命令来封装:

.. code-block:: python

  import subprocess
  import sys
  # for simplicity we do not check number of
  # arguments and whether the file really exists
  file_path = sys.argv[-1]
  try:
      output = subprocess.check_output(['size', file_path]).decode('utf-8')
  except FileNotFoundError:
      print('command "size" is not available on this platform')
      sys.exit(0)
  size = 0.0
  for line in output.split('\n'):
      if file_path in line:
          # we are interested in the 4th number on this line
          size = int(line.split()[3])
  print('{0:.3f} MB'.format(size/1.0e6))

要打印链接行，我们将使用第二个Python helper脚本(echo-file.py)打印文件的内容:

.. code-block:: python

  import sys
  # for simplicity we do not verify the number and
  # type of arguments
  file_path = sys.argv[-1]
  try:
      with open(file_path, 'r') as f:
  print(f.read())
  except FileNotFoundError:
      print('ERROR: file {0} not found'.format(file_path))

**具体实施**

来看看CMakeLists.txt：

1 首先声明一个Fortran项目:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-05 LANGUAGES Fortran)

2 例子依赖于Python解释器，所以以一种可移植的方式执行helper脚本:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)

3 本例中，默认为“Release”构建类型，以便CMake添加优化标志:

.. code-block:: cmake

  if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()

4 现在，定义可执行目标:

.. code-block:: cmake

  add_executable(example "")
  target_sources(example
    PRIVATE
        example.f90
    )

5 然后，定义一个自定义命令，在example目标在已链接之前，打印链接行:

.. code-block:: cmake

  add_custom_command(
    TARGET
        example
    PRE_LINK
        COMMAND
            ${PYTHON_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/echo-file.py
              ${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/example.dir/link.txt
    COMMENT
        "link line:"
    VERBATIM
    )

6 测试一下。观察打印的链接行和可执行文件的静态大小:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  Scanning dependencies of target example
  [ 50%] Building Fortran object CMakeFiles/example.dir/example.f90.o
  [100%] Linking Fortran executable example
  link line:
  /usr/bin/f95 -O3 -DNDEBUG -O3 CMakeFiles/example.dir/example.f90.o -o example
  static size of executable:
  160.003 MB
  [100%] Built target example

**工作原理**

当声明了库或可执行目标，就可以使用add_custom_command将其他命令锁定到目标上。这些命令将在特定的时间执行，与它们所附加的目标的执行相关联。
CMake通过以下选项，定制命令执行顺序:

* PRE_BUILD：在执行与目标相关的任何其他规则之前执行的命令。
* PRE_LINK：使用此选项，命令在编译目标之后，调用链接器或归档器之前执行。Visual Studio 7或更高版本之外的生成器中使用PRE_BUILD将被解释为PRE_LINK。
* POST_BUILD：如前所述，这些命令将在执行给定目标的所有规则之后运行。

本例中，将两个自定义命令绑定到可执行目标。PRE_LINK命令将${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/example.dir/link.txt的内容打印到屏幕上。
在我们的例子中，链接行是这样的:

.. code-block:: bash

  link line:
  /usr/bin/f95 -O3 -DNDEBUG -O3 CMakeFiles/example.dir/example.f90.o -o example

使用Python包装器来实现这一点，它依赖于shell命令。

第二步中，POST_BUILD自定义命令调用Python helper脚本static-size.py，生成器表达式$<target_file:example>作为参数。
CMake将在生成时(即生成生成系统时)将生成器表达式扩展到目标文件路径。然后，Python脚本static-size.py使用size命令获取可执行文件的静态分配大小，
将其转换为MB，并打印结果。我们的例子中，获得了预期的160 MB:

.. code-block:: bash

  static size of executable:
  160.003 MB

5.6 探究编译和链接命令
---------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-06 中找到，其中包含一个C++例子。
  该示例在CMake 3.9版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。代码库还有一个与CMake 3.5兼容的示例。

生成构建系统期间最常见的操作，是试图评估在哪种系统上构建项目。这意味着要找出哪些功能工作，哪些不工作，并相应地调整项目的编译。
使用的方法是查询依赖项是否被满足的信号，或者在代码库中是否启用工作区。接下来的几个示例，将展示如何使用CMake执行这些操作。我们将特别讨论以下事宜:

1 如何确保代码能成功编译为可执行文件。
2 如何确保编译器理解相应的标志。
3 如何确保特定代码能成功编译为运行可执行程序。

**准备工作**

示例将展示如何使用来自对应的Check<LANG>SourceCompiles.cmake标准模块的check_<lang>_source_compiles函数，
以评估给定编译器是否可以将预定义的代码编译成可执行文件。该命令可帮助你确定:

* 编译器支持所需的特性。
* 链接器工作正常，并理解特定的标志。
* 可以使用find_package找到的包含目录和库。

本示例中，我们将展示如何检测OpenMP 4.5标准的循环特性，以便在C++可执行文件中使用。使用一个C++源文件，来探测编译器是否支持这样的特性。
CMake提供了一个附加命令try_compile来探究编译。本示例将展示，如何使用这两种方法。

TIPS: 可以使用CMake命令行界面来获取关于特定模块(cmake --help-module <module-name>)和命令(cmake --help-command <command-name>)的文档。
示例中，cmake --help-module CheckCXXSourceCompiles将把check_cxx_source_compiles函数的文档输出到屏幕上，
而cmake --help-command try_compile将对try_compile命令执行相同的操作。

**具体实施**

我们将同时使用try_compile和check_cxx_source_compiles，并比较这两个命令的工作方式:

1 创建一个C++11工程：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
  project(recipe-06 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 查找编译器支持的OpenMP：

.. code-block:: cmake

  find_package(OpenMP)
  if(OpenMP_FOUND)
      # ... <- the steps below will be placed here
  else()
      message(STATUS "OpenMP not found: no test for taskloop is run")
  endif()

3 如果找到OpenMP，再检查所需的特性是否可用。为此，设置了一个临时目录，try_compile将在这个目录下来生成中间文件。
  我们把它放在前面步骤中引入的if语句中:

.. code-block:: cmake

  set(_scratch_dir ${CMAKE_CURRENT_BINARY_DIR}/omp_try_compile)

4 调用try_compile生成一个小项目，以尝试编译源文件taskloop.cpp。编译成功或失败的状态，将保存到omp_taskloop_test_1变量中。
  需要为这个示例编译设置适当的编译器标志、包括目录和链接库。因为使用导入的目标OpenMP::OpenMP_CXX，
  所以只需将LINK_LIBRARIES选项设置为try_compile即可。如果编译成功，则任务循环特性可用，我们为用户打印一条消息:

.. code-block:: cmake

  try_compile(
    omp_taskloop_test_1
        ${_scratch_dir}
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/taskloop.cpp
    LINK_LIBRARIES
        OpenMP::OpenMP_CXX
    )
  message(STATUS "Result of try_compile: ${omp_taskloop_test_1}")

5 要使用check_cxx_source_compiles函数，需要包含CheckCXXSourceCompiles.cmake模块文件。其他语言也有类似的模块文件，C(CheckCSourceCompiles.cmake)和Fortran(CheckFortranSourceCompiles.cmake):

.. code-block:: cmake

  include(CheckCXXSourceCompiles)

6 我们复制源文件的内容，通过file(READ ...)命令读取内容到一个变量中，试图编译和连接这个变量:

.. code-block:: cmake

  file(READ ${CMAKE_CURRENT_SOURCE_DIR}/taskloop.cpp _snippet)

7 我们设置了CMAKE_REQUIRED_LIBRARIES。这对于下一步正确调用编译器是必需的。注意使用导入的OpenMP::OpenMP_CXX目标，
  它还将设置正确的编译器标志和包含目录:

.. code-block:: cmake

  set(CMAKE_REQUIRED_LIBRARIES OpenMP::OpenMP_CXX)

8 使用代码片段作为参数，调用check_cxx_source_compiles函数。检查结果将保存到omp_taskloop_test_2变量中:

.. code-block:: cmake

  check_cxx_source_compiles("${_snippet}" omp_taskloop_test_2)

9 调用check_cxx_source_compiles并向用户打印消息之前，我们取消了变量的设置:

.. code-block:: cmake

  unset(CMAKE_REQUIRED_LIBRARIES)
  message(STATUS "Result of check_cxx_source_compiles: ${omp_taskloop_test_2}"

10 最后，进行测试：

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  -- ...
  -- Found OpenMP_CXX: -fopenmp (found version "4.5")
  -- Found OpenMP: TRUE (found version "4.5")
  -- Result of try_compile: TRUE
  -- Performing Test omp_taskloop_test_2
  -- Performing Test omp_taskloop_test_2 - Success
  -- Result of check_cxx_source_compiles: 1

**工作原理**

try_compile和check_cxx_source_compiles都将编译源文件，并将其链接到可执行文件中。如果这些操作成功，那么输出变量omp_task_loop_test_1(前者)
和omp_task_loop_test_2(后者)将被设置为TRUE。然而，这两个命令实现的方式略有不同。check_<lang>_source_compiles命令是try_compile命令的简化包装。
因此，它提供了一个接口:

1 要编译的代码片段必须作为CMake变量传入。大多数情况下，这意味着必须使用file(READ ...)来读取文件。然后，
代码片段被保存到构建目录的CMakeFiles/CMakeTmp子目录中。

2 微调编译和链接，必须通过设置以下CMake变量进行:

* CMAKE_REQUIRED_FLAGS：设置编译器标志。
* CMAKE_REQUIRED_DEFINITIONS：设置预编译宏。
* CMAKE_REQUIRED_INCLUDES：设置包含目录列表。
* CMAKE_REQUIRED_LIBRARIES：设置可执行目标能够连接的库列表。

3 调用check_<lang>_compiles_function之后，必须手动取消对这些变量的设置，以确保后续使用中，不会保留当前内容。

.. NOTE::

  使用CMake 3.9中可以对于OpenMP目标进行导入,但是目前的配置也可以使用CMake的早期版本，通过手动为check_cxx_source_compiles设置所需的标志和库:
  set(CMAKE_REQUIRED_FLAGS ${OpenMP_CXX_FLAGS})和set(CMAKE_REQUIRED_LIBRARIES ${OpenMP_CXX_LIBRARIES})。

这个接口反映了：测试编译是通过，在CMake调用中直接生成和执行构建和连接命令来执行的。

命令try_compile提供了更完整的接口和两种不同的操作模式:

1 以一个完整的CMake项目作为输入，并基于它的CMakeLists.txt配置、构建和链接。这种操作模式提供了更好的灵活性，因为要编译项目的复杂度是可以选择的。

2 提供了源文件，和用于包含目录、链接库和编译器标志的配置选项。

因此，try_compile基于在项目上调用CMake，其中CMakeLists.txt已经存在(在第一种操作模式中)，或者基于传递给try_compile的参数动态生成文件。

**更多信息**

本示例中概述的类型检查并不总是万无一失的，并且可能产生假阳性和假阴性。作为一个例子，可以尝试注释掉包含CMAKE_REQUIRED_LIBRARIES的行。
运行这个例子仍然会报告“成功”，这是因为编译器将忽略OpenMP的pragma字段。

当返回了错误的结果时，应该怎么做？构建目录的CMakeFiles子目录中的CMakeOutput.log和CMakeError.log文件会提供一些线索。
它们记录了CMake运行的操作的标准输出和标准错误。如果怀疑结果有误，应该通过搜索保存编译检查结果的变量集来检查前者。如果你怀疑有误报，你应该检查后者。

调试try_compile需要一些注意事项。即使检查不成功，CMake也会删除由该命令生成的所有文件。幸运的是，debug-trycompile将阻止CMake进行删除。
如果你的代码中有多个try_compile调用，一次只能调试一个:

1 运行CMake，不使用--debug-trycompile，将运行所有try_compile命令，并清理它们的执行目录和文件。

2 从CMake缓存中删除保存检查结果的变量。缓存保存到CMakeCache.txt文件中。要清除变量的内容，可以使用-U的CLI开关，后面跟着变量的名称，
它将被解释为一个全局表达式，因此可以使用*和?：

.. code-block:: bash

    $ cmake -U <variable-name>


3 再次运行CMake，使用--debug-trycompile。只有清除缓存的检查才会重新运行。这次不会清理执行目录和文件。

5.7 探究编译器标志命令
---------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-07 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

设置编译器标志，对是否能正确编译至关重要。不同的编译器供应商，为类似的特性实现有不同的标志。即使是来自同一供应商的不同编译器版本，在可用标志上也可能存在细微的差异。有时，会引入一些便于调试或优化目的的新标志。本示例中，我们将展示如何检查所选编译器是否可用某些标志。

**准备工作**

Sanitizers(请参考https://github.com/google/Sanitizers )已经成为静态和动态代码分析的非常有用的工具。
通过使用适当的标志重新编译代码并链接到必要的库，可以检查内存错误(地址清理器)、未初始化的读取(内存清理器)、线程安全(线程清理器)和未定义的行为
(未定义的行为清理器)相关的问题。与同类型分析工具相比，Sanitizers带来的性能损失通常要小得多，而且往往提供关于检测到的问题的更详细的信息。
缺点是，代码(可能还有工具链的一部分)需要使用附加的标志重新编译。

本示例中，我们将设置一个项目，使用不同的Sanitizers来编译代码，并展示如何检查，编译器标志是否正确使用。

**具体实施**

Clang编译器已经提供了Sanitizers，GCC也将其引入工具集中。它们是为C和C++程序而设计的。最新版本的Fortran也能使用这些编译标志，并生成正确的仪表化库和可执行程序。不过，本文将重点介绍C++示例。

1 声明一个C++11项目：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-07 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 声明列表CXX_BASIC_FLAGS，其中包含构建项目时始终使用的编译器标志-g3和-O1:

.. code-block:: cmake

  list(APPEND CXX_BASIC_FLAGS "-g3" "-O1")

3 这里需要包括CMake模块CheckCXXCompilerFlag.cmake。C的模块为CheckCCompilerFlag.cmake，
  Fotran的模块为CheckFortranCompilerFlag.cmake(Fotran的模块是在CMake 3.3添加)：

.. code-block:: cmake

  include(CheckCXXCompilerFlag)

4 我们声明一个ASAN_FLAGS变量，它包含Sanitizer所需的标志，并设置CMAKE_REQUIRED_FLAGS变量，check_cxx_compiler_flag函数在内部使用该变量:

.. code-block:: cmake

  set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer")
  set(CMAKE_REQUIRED_FLAGS ${ASAN_FLAGS})

5 我们调用check_cxx_compiler_flag来确保编译器理解ASAN_FLAGS变量中的标志。调用函数后，我们取消设置CMAKE_REQUIRED_FLAGS:

.. code-block:: cmake

  check_cxx_compiler_flag(${ASAN_FLAGS} asan_works)
  unset(CMAKE_REQUIRED_FLAGS)

6 如果编译器理解这些选项，我们将变量转换为一个列表，用分号替换空格:

.. code-block:: cmake

  if(asan_works)
      string(REPLACE " " ";" _asan_flags ${ASAN_FLAGS})

7 我们添加了一个可执行的目标，为代码定位Sanitizer:

.. code-block:: cmake

  add_executable(asan-example asan-example.cpp)

8 我们为可执行文件设置编译器标志，以包含基本的和Sanitizer标志:

.. code-block:: cmake

  target_compile_options(asan-example
    PUBLIC
      ${CXX_BASIC_FLAGS}
      ${_asan_flags}
    )

9 最后，我们还将Sanitizer标志添加到链接器使用的标志集中。这将关闭if(asan_works)块:

.. code-block:: cmake

  target_link_libraries(asan-example PUBLIC ${_asan_flags})
  endif()

完整的示例源代码还展示了如何编译和链接线程、内存和未定义的行为清理器的示例可执行程序。这里不详细讨论这些，因为我们使用相同的模式来检查编译器标志。

.. NOTE:: 

  在GitHub上可以找到一个定制的CMake模块，用于在您的系统上寻找对Sanitizer的支持:https://github.com/arsenm/sanitizers-cmake

**工作原理**

check_<lang>_compiler_flag函数只是check_<lang>_source_compiles函数的包装器。这些包装器为特定代码提供了一种快捷方式。在用例中，
检查特定代码片段是否编译并不重要，重要的是编译器是否理解一组标志。

Sanitizer的编译器标志也需要传递给链接器。可以使用check_<lang>_compiler_flag函数来实现，我们需要在调用之前设置CMAKE_REQUIRED_FLAGS变量。
否则，作为第一个参数传递的标志将只对编译器使用。

当前配置中需要注意的是，使用字符串变量和列表来设置编译器标志。使用target_compile_options和target_link_libraries函数的字符串变量，
将导致编译器和/或链接器报错。CMake将传递引用的这些选项，从而导致解析错误。这说明有必要用列表和随后的字符串操作来表示这些选项，
并用分号替换字符串变量中的空格。实际上，CMake中的列表是分号分隔的字符串。

5.8 探究可执行命令
---------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-08 中找到，其中包含一个C/C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

目前为止，我们已经展示了如何检查给定的源代码，是否可以由所选的编译器编译，以及如何确保所需的编译器和链接器标志可用。此示例中，
将显示如何检查是否可以在当前系统上编译、链接和运行代码。

**准备工作**

本示例的代码示例是复用第3章第9节的配置，并进行微小的改动。之前，我们展示了如何在您的系统上找到ZeroMQ库并将其链接到一个C程序中。
本示例中，在生成实际的C++程序之前，我们将检查一个使用GNU/Linux上的系统UUID库的小型C程序是否能够实际运行。


**具体实施**

开始构建C++项目之前，我们希望检查GNU/Linux上的UUID系统库是否可以被链接。这可以通过以下一系列步骤来实现:

1 声明一个混合的C和C++11程序。这是必要的，因为我们要编译和运行的测试代码片段是使用C语言完成:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
  project(recipe-08 LANGUAGES CXX C)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 我们需要在系统上找到UUID库。这通过使用pkg-config实现的。要求搜索返回一个CMake导入目标使用IMPORTED_TARGET参数:

.. code-block:: cmake

  find_package(PkgConfig REQUIRED QUIET)
  pkg_search_module(UUID REQUIRED uuid IMPORTED_TARGET)
  if(TARGET PkgConfig::UUID)
      message(STATUS "Found libuuid")
  endif()

3 接下来，需要使用CheckCSourceRuns.cmake模块。C++的是CheckCXXSourceRuns.cmake模块。但到CMake 3.11为止，Fortran语言还没有这样的模块:

.. code-block:: cmake

  include(CheckCSourceRuns)

4 我们声明一个_test_uuid变量，其中包含要编译和运行的C代码段:

.. code-block:: c++

  set(_test_uuid
  "
  #include <uuid/uuid.h>
  int main(int argc, char * argv[]) {
    uuid_t uuid;
    uuid_generate(uuid);
    return 0;
  }
  ")

5 我们声明CMAKE_REQUIRED_LIBRARIES变量后，对check_c_source_runs函数的调用。接下来，调用check_c_source_runs，
  其中测试代码作为第一个参数，_runs变量作为第二个参数，以保存执行的检查结果。之后，取消CMAKE_REQUIRED_LIBRARIES变量的设置:

.. code-block:: cmake

  set(CMAKE_REQUIRED_LIBRARIES PkgConfig::UUID)
  check_c_source_runs("${_test_uuid}" _runs)
  unset(CMAKE_REQUIRED_LIBRARIES)

6 如果检查没有成功，要么是代码段没有编译，要么是没有运行，我们会用致命的错误停止配置:

.. code-block:: cmake

  if(NOT _runs)
      message(FATAL_ERROR "Cannot run a simple C executable using libuuid!")
  endif()

7 若成功，我们继续添加C++可执行文件作为目标，并链接到UUID:

.. code-block:: cmake

  add_executable(use-uuid use-uuid.cpp)
  target_link_libraries(use-uuid
    PUBLIC
        PkgConfig::UUID
    )

**工作原理**

check_<lang>_source_runs用于C和C++的函数，与check_<lang>_source_compile相同，但在实际运行生成的可执行文件的地方需要添加一个步骤。
对于check_<lang>_source_compiles, check_<lang>_source_runs的执行可以通过以下变量来进行:

* CMAKE_REQUIRED_FLAGS：设置编译器标志。
* CMAKE_REQUIRED_DEFINITIONS：设置预编译宏。
* CMAKE_REQUIRED_INCLUDES：设置包含目录列表。
* CMAKE_REQUIRED_LIBRARIES：设置可执行目标需要连接的库列表。

由于使用pkg_search_module生成的为导入目标，所以只需要将CMAKE_REQUIRES_LIBRARIES设置为PkgConfig::UUID，就可以正确设置包含目录。

正如check_<lang>_source_compiles是try_compile的包装器，check_<lang>_source_runs是CMake中另一个功能更强大的命令的包装器:try_run。因此，可以编写一个CheckFortranSourceRuns.cmake模块，通过适当包装try_run, 提供与C和C++模块相同的功能。

.. NOTE::

  pkg_search_module只能定义导入目标(CMake 3.6),但目前的示例可以使工作，3.6之前版本的CMake可以通过手动设置所需的包括目录和库
  check_c_source_runs如下:set(CMAKE_REQUIRED_INCLUDES $ {UUID_INCLUDE_DIRS})和set(CMAKE_REQUIRED_LIBRARIES $ {UUID_LIBRARIES})。

5.9 使用生成器表达式微调配置和编译
---------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-5/recipe-09 中找到，其中包含一个C++例子。
  该示例在CMake 3.9版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

CMake提供了一种特定于领域的语言，来描述如何配置和构建项目。自然会引入描述特定条件的变量，并在CMakeLists.txt中包含基于此的条件语句。

本示例中，我们将重新讨论生成器表达式。第4章中，以简洁地引用显式的测试可执行路径，使用了这些表达式。生成器表达式为逻辑和信息表达式，
提供了一个强大而紧凑的模式，这些表达式在生成构建系统时进行评估，并生成特定于每个构建配置的信息。换句话说，生成器表达式用于引用仅在生成时已知，
但在配置时未知或难于知晓的信息；对于文件名、文件位置和库文件后缀尤其如此。

本例中，我们将使用生成器表达式，有条件地设置预处理器定义，并有条件地链接到消息传递接口库(Message Passing Interface, MPI)，
并允许我们串行或使用MPI构建相同的源代码。


**准备工作**

我们将编译以下示例源代码(example.cpp):

.. code-block:: c++

  #include <iostream>
  #ifdef HAVE_MPI
  #include <mpi.h>
  #endif
  int main()
  {
  #ifdef HAVE_MPI
    // initialize MPI
    MPI_Init(NULL, NULL);
    // query and print the rank
    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    std::cout << "hello from rank " << rank << std::endl;
    // initialize MPI
    MPI_Finalize();
  #else
    std::cout << "hello from a sequential binary" << std::endl;
  #endif /* HAVE_MPI */
  }


代码包含预处理语句(#ifdef HAVE_MPI ... #else ... #endif)，这样我们就可以用相同的源代码编译一个顺序的或并行的可执行文件了。

**具体实施**

编写CMakeLists.txt文件时，我们将重用第3章第6节的一些构建块:

1 声明一个C++11项目：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
  project(recipe-09 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 然后，我们引入一个选项USE_MPI来选择MPI并行化，并将其设置为默认值ON。如果为ON，我们使用find_package来定位MPI环境:

.. code-block:: cmake

  option(USE_MPI "Use MPI parallelization" ON)
  if(USE_MPI)
      find_package(MPI REQUIRED)
  endif()

3 然后定义可执行目标，并有条件地设置相应的库依赖项(MPI::MPI_CXX)和预处理器定义(HAVE_MPI)，稍后将对此进行解释:

.. code-block:: cmake

  add_executable(example example.cpp)
  target_link_libraries(example
    PUBLIC
        $<$<BOOL:${MPI_FOUND}>:MPI::MPI_CXX>
    )
  target_compile_definitions(example
    PRIVATE
        $<$<BOOL:${MPI_FOUND}>:HAVE_MPI>
    )

4 如果找到MPI，还将打印由FindMPI.cmake导出的INTERFACE_LINK_LIBRARIES，为了方便演示，使用了cmake_print_properties()函数:

.. code-block:: cmake

  if(MPI_FOUND)
    include(CMakePrintHelpers)
    cmake_print_properties(
      TARGETS MPI::MPI_CXX
      PROPERTIES INTERFACE_LINK_LIBRARIES
      )
  endif()

5 首先使用默认MPI配置。观察cmake_print_properties()的输出:

.. code-block:: bash

  $ mkdir -p build_mpi
  $ cd build_mpi
  $ cmake ..
  -- ...
  --
  Properties for TARGET MPI::MPI_CXX:
  MPI::MPI_CXX.INTERFACE_LINK_LIBRARIES = "-Wl,-rpath -Wl,/usr/lib/openmpi -Wl,--enable-new-dtags -pthread;/usr/lib/openmpi/libmpi_cxx.so;/usr/lib/openmpi/libmpi.so"

6 编译并运行并行例子:

.. code-block:: bash

  $ cmake --build .
  $ mpirun -np 2 ./example
  hello from rank 0
  hello from rank 1

7 现在，创建一个新的构建目录，这次构建串行版本:

.. code-block:: bash

  $ mkdir -p build_seq
  $ cd build_seq
  $ cmake -D USE_MPI=OFF ..
  $ cmake --build .
  $ ./example
  hello from a sequential binary

**工作原理**

CMake分两个阶段生成项目的构建系统：配置阶段(解析CMakeLists.txt)和生成阶段(实际生成构建环境)。生成器表达式在第二阶段进行计算，
可以使用仅在生成时才能知道的信息来调整构建系统。生成器表达式在交叉编译时特别有用，一些可用的信息只有解析CMakeLists.txt之后，或在多配置项目后获取，
构建系统生成的所有项目可以有不同的配置，比如Debug和Release。

本例中，将使用生成器表达式有条件地设置链接依赖项并编译定义。为此，可以关注这两个表达式:

.. code-block:: cmake

  target_link_libraries(example
    PUBLIC
        $<$<BOOL:${MPI_FOUND}>:MPI::MPI_CXX>
    )
  target_compile_definitions(example
    PRIVATE
        $<$<BOOL:${MPI_FOUND}>:HAVE_MPI>
    )

如果MPI_FOUND为真，那么$<BOOL:${MPI_FOUND}>的值将为1。本例中，$<$<BOOL:${MPI_FOUND}>:MPI::MPI_CXX>将计算MPI::MPI_CXX，
第二个生成器表达式将计算结果存在HAVE_MPI。如果将USE_MPI设置为OFF，则MPI_FOUND为假，两个生成器表达式的值都为空字符串，
因此不会引入链接依赖关系，也不会设置预处理定义。

我们可以通过if来达到同样的效果:

.. code-block:: cmake

  if(MPI_FOUND)
    target_link_libraries(example
      PUBLIC
          MPI::MPI_CXX
      )
    target_compile_definitions(example
      PRIVATE
          HAVE_MPI
      )
  endif()

这个解决方案不太优雅，但可读性更好。我们可以使用生成器表达式来重新表达if语句，而这个选择取决于个人喜好。但当我们需要访问或操作文件路径时，
生成器表达式尤其出色，因为使用变量和if构造这些路径可能比较困难。本例中，我们更注重生成器表达式的可读性。第4章中，
我们使用生成器表达式来解析特定目标的文件路径。第11章中，我们会再次来讨论生成器。

**更多信息**

CMake提供了三种类型的生成器表达式:

* 逻辑表达式，基本模式为$<condition:outcome>。基本条件为0表示false, 1表示true，但是只要使用了正确的关键字，任何布尔值都可以作为条件变量。
* 信息表达式，基本模式为$<information>或$<information:input>。这些表达式对一些构建系统信息求值，例如：包含目录、目标属性等等。
  这些表达式的输入参数可能是目标的名称，比如表达式$<TARGET_PROPERTY:tgt,prop>，将获得的信息是tgt目标上的prop属性。
* 输出表达式，基本模式为$<operation>或$<operation:input>。这些表达式可能基于一些输入参数，生成一个输出。它们的输出可以直接在CMake命令中使用，
  也可以与其他生成器表达式组合使用。例如,- I$<JOIN:$<TARGET_PROPERTY:INCLUDE_DIRECTORIES>, -I>将生成一个字符串，
  其中包含正在处理的目标的包含目录，每个目录的前缀由-I表示。

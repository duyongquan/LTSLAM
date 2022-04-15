.. highlight:: c++

.. default-domain:: cpp

==========================
第6章 生成源码
==========================

6.1 配置时生成源码
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-6/recipe-01 中找到，其中包含一个Fortran/C例子。
  该示例在CMake 3.10版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows(使用MSYS Makefiles)上进行过测试。

代码生成在配置时发生，例如：CMake可以检测操作系统和可用库；基于这些信息，我们可以定制构建的源代码。本节和下面的章节中，我们将演示如何生成一个简单源文件，
该文件定义了一个函数，用于报告构建系统配置。

**准备工作**

此示例的代码使用Fortran和C语言编写，第9章将讨论混合语言编程。主程序是一个简单的Fortran可执行程序，它调用一个C函数print_info()，该函数将打印配置信息。值得注意的是，在使用Fortran 2003时，编译器将处理命名问题(对于C函数的接口声明)，如示例所示。我们将使用的example.f90作为源文件:

.. code-block:: bash

  program hello_world
    implicit none
    interface
        subroutine print_info() bind(c, name="print_info")
        end subroutine
    end interface
    call print_info()
  end program

C函数print_info()在模板文件print_info.c.in中定义。在配置时，以@开头和结尾的变量将被替换为实际值:

.. code-block:: c++

  #include <stdio.h>
  #include <unistd.h>
  void print_info(void)
  {
    printf("\n");
    printf("Configuration and build information\n");
    printf("-----------------------------------\n");
    printf("\n");
    printf("Who compiled | %s\n", "@_user_name@");
    printf("Compilation hostname | %s\n", "@_host_name@");
    printf("Fully qualified domain name | %s\n", "@_fqdn@");
    printf("Operating system | %s\n",
          "@_os_name@, @_os_release@, @_os_version@");
    printf("Platform | %s\n", "@_os_platform@");
    printf("Processor info | %s\n",
          "@_processor_name@, @_processor_description@");
    printf("CMake version | %s\n", "@CMAKE_VERSION@");
    printf("CMake generator | %s\n", "@CMAKE_GENERATOR@");
    printf("Configuration time | %s\n", "@_configuration_time@");
    printf("Fortran compiler | %s\n", "@CMAKE_Fortran_COMPILER@");
    printf("C compiler | %s\n", "@CMAKE_C_COMPILER@");
    printf("\n");
    fflush(stdout);
  }

**具体实施**

在CMakeLists.txt中，我们首先必须对选项进行配置，并用它们的值替换print_info.c.in中相应的占位符。然后，将Fortran和C源代码编译成一个可执行文件:

1 声明了一个Fortran-C混合项目:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
  project(recipe-01 LANGUAGES Fortran C)

2 使用execute_process为项目获取当且使用者的信息:

.. code-block:: cmake

  execute_process(
    COMMAND
        whoami
    TIMEOUT
        1
    OUTPUT_VARIABLE
        _user_name
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

3 使用cmake_host_system_information()函数(已经在第2章第5节遇到过)，可以查询很多系统信息:

.. code-block:: cmake

  # host name information
  cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)
  cmake_host_system_information(RESULT _fqdn QUERY FQDN)
  # processor information
  cmake_host_system_information(RESULT _processor_name QUERY PROCESSOR_NAME)
  cmake_host_system_information(RESULT _processor_description QUERY PROCESSOR_DESCRIPTION)
  # os information
  cmake_host_system_information(RESULT _os_name QUERY OS_NAME)
  cmake_host_system_information(RESULT _os_release QUERY OS_RELEASE)
  cmake_host_system_information(RESULT _os_version QUERY OS_VERSION)
  cmake_host_system_information(RESULT _os_platform QUERY OS_PLATFORM)

4 捕获配置时的时间戳，并通过使用字符串操作函数:

.. code-block:: cmake

  string(TIMESTAMP _configuration_time "%Y-%m-%d %H:%M:%S [UTC]" UTC)

5 现在，准备好配置模板文件print_info.c.in。通过CMake的configure_file函数生成代码。注意，这里只要求以@开头和结尾的字符串被替换:

.. code-block:: cmake

  configure_file(print_info.c.in print_info.c @ONLY)

6 最后，我们添加一个可执行目标，并定义目标源：

.. code-block:: cmake

  add_executable(example "")
  target_sources(example
    PRIVATE
      example.f90
      ${CMAKE_CURRENT_BINARY_DIR}/print_info.c
    )

7 下面是一个输出示例：

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./example
  Configuration and build information
  -----------------------------------
  Who compiled | somebody
  Compilation hostname | laptop
  Fully qualified domain name | laptop
  Operating system | Linux, 4.16.13-1-ARCH, #1 SMP PREEMPT Thu May 31 23:29:29 UTC 2018
  Platform | x86_64
  Processor info | Unknown P6 family, 2 core Intel(R) Core(TM) i5-5200U CPU @ 2.20GHz
  CMake version | 3.11.3
  CMake generator | Unix Makefiles
  Configuration time | 2018-06-25 15:38:03 [UTC]
  Fortran compiler | /usr/bin/f95
  C compiler | /usr/bin/cc

**工作原理**

configure_file命令可以复制文件，并用变量值替换它们的内容。示例中，使用configure_file修改模板文件的内容，并将其复制到一个位置，
然后将其编译到可执行文件中。如何调用configure_file:

.. code-block:: cmake

  configure_file(print_info.c.in print_info.c @ONLY)

第一个参数是模板的名称为print_info.c.in。CMake假设输入文件的目录，与项目的根目录相对；也就是说，在${CMAKE_CURRENT_SOURCE_DIR}/print_info.c.in。
我们选择print_info.c，作为第二个参数是配置文件的名称。假设输出文件位于相对于项目构建目录的位置：${CMAKE_CURRENT_BINARY_DIR}/print_info.c。

输入和输出文件作为参数时，CMake不仅将配置@VAR@变量，还将配置${VAR}变量。如果${VAR}是语法的一部分，并且不应该修改(例如在shell脚本中)，那么就很不方便。
为了在引导CMake，应该将选项@ONLY传递给configure_file的调用，如前所述。

6.2 使用Python在配置时生成源码
------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-6/recipe-02 中找到，其中包含一个Fortran/C例子。
  该示例在CMake 3.10版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows(使用MSYS Makefile)上进行过测试。

本示例中，我们将再次从模板print_info.c.in生成print_info.c。但这一次，将假设CMake函数configure_file()没有创建源文件，
然后使用Python脚本模拟这个过程。当然，对于实际的项目，我们可能更倾向于使用configure_file()，但有时使用Python生成源代码的需要时，
我们也应该知道如何应对。

这个示例有严重的限制，不能完全模拟configure_file()。我们在这里介绍的方法，不能生成一个自动依赖项，该依赖项将在构建时重新生成print_info.c。
换句话说，如果在配置之后删除生成的print_info.c，则不会重新生成该文件，构建也会失败。要正确地模拟configure_file()，
需要使用add_custom_command()和add_custom_target()。我们将在第3节中使用它们，来克服这个限制。

这个示例中，我们将使用一个简单的Python脚本。这个脚本将读取print_info.c.in。用从CMake传递给Python脚本的参数替换文件中的占位符。
对于更复杂的模板，我们建议使用外部工具，比如Jinja(参见http://jinja.pocoo.org )。

.. code-block:: python

  def configure_file(input_file, output_file, vars_dict):
    with input_file.open('r') as f:
        template = f.read()
    for var in vars_dict: 
        template = template.replace('@' + var + '@', vars_dict[var])
    with output_file.open('w') as f:
        f.write(template)

这个函数读取一个输入文件，遍历vars_dict变量中的目录，并用对应的值替换@key@，再将结果写入输出文件。这里的键值对，将由CMake提供。

**准备工作**

print_info.c.in和example.f90与之前的示例相同。此外，我们将使用Python脚本configuration.py，它提供了一个函数:

.. code-block:: python

  def configure_file(input_file, output_file, vars_dict):
    with input_file.open('r') as f:
        template = f.read()
    for var in vars_dict:
        template = template.replace('@' + var + '@', vars_dict[var])
    with output_file.open('w') as f:
        f.write(template)

该函数读取输入文件，遍历vars_dict字典的所有键，用对应的值替换模式@key@，并将结果写入输出文件(键值由CMake提供)。

**具体实施**

与前面的示例类似，我们需要配置一个模板文件，但这一次，使用Python脚本模拟configure_file()函数。我们保持CMakeLists.txt基本不变，
并提供一组命令进行替换操作configure_file(print_info.c.in print_info.c @ONLY)，接下来将逐步介绍这些命令:

1 首先，构造一个变量_config_script，它将包含一个Python脚本，稍后我们将执行这个脚本:

.. code-block:: cmake

  set(_config_script
  "
  from pathlib import Path
  source_dir = Path('${CMAKE_CURRENT_SOURCE_DIR}')
  binary_dir = Path('${CMAKE_CURRENT_BINARY_DIR}')
  input_file = source_dir / 'print_info.c.in'
  output_file = binary_dir / 'print_info.c'
  import sys
  sys.path.insert(0, str(source_dir))
  from configurator import configure_file
  vars_dict = {
    '_user_name': '${_user_name}',
    '_host_name': '${_host_name}',
    '_fqdn': '${_fqdn}',
    '_processor_name': '${_processor_name}',
    '_processor_description': '${_processor_description}',
    '_os_name': '${_os_name}',
    '_os_release': '${_os_release}',
    '_os_version': '${_os_version}',
    '_os_platform': '${_os_platform}',
    '_configuration_time': '${_configuration_time}',
    'CMAKE_VERSION': '${CMAKE_VERSION}',
    'CMAKE_GENERATOR': '${CMAKE_GENERATOR}',
    'CMAKE_Fortran_COMPILER': '${CMAKE_Fortran_COMPILER}',
    'CMAKE_C_COMPILER': '${CMAKE_C_COMPILER}',
  }
  configure_file(input_file, output_file, vars_dict)
  ")

2 使用find_package让CMake使用Python解释器:

.. code-block:: cmake

  find_package(PythonInterp QUIET REQUIRED)

3 如果找到Python解释器，则可以在CMake中执行_config_script，并生成print_info.c文件:

.. code-block:: cmake

  execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} "-c" ${_config_script}
    )

4 之后，定义可执行目标和依赖项，这与前一个示例相同。所以，得到的输出没有变化。

**工作原理**

回顾一下对CMakeLists.txt的更改。

我们执行了一个Python脚本生成print_info.c。运行Python脚本前，首先检测Python解释器，并构造Python脚本。Python脚本导入configure_file函数，
我们在configuration.py中定义了这个函数。为它提供用于读写的文件位置，并将其值作为键值对。

此示例展示了生成配置的另一种方法，将生成任务委托给外部脚本，可以将配置报告编译成可执行文件，甚至库目标。我们在前面的配置中认为的第一种方法更简洁，
但是使用本示例中提供的方法，我们可以灵活地使用Python(或其他语言)，实现任何在配置时间所需的步骤。使用当前方法，
我们可以通过脚本的方式执行类似cmake_host_system_information()的操作。

但要记住，这种方法也有其局限性，它不能在构建时重新生成print_info.c的自动依赖项。下一个示例中，我们应对这个挑战
首先，构造一个变量_config_script，它将包含一个Python脚本，稍后我们将执行这个脚本:

.. code-block:: cmake

  set(_config_script
  "
  from pathlib import Path
  source_dir = Path('${CMAKE_CURRENT_SOURCE_DIR}')
  binary_dir = Path('${CMAKE_CURRENT_BINARY_DIR}')
  input_file = source_dir / 'print_info.c.in'
  output_file = binary_dir / 'print_info.c'
  import sys
  sys.path.insert(0, str(source_dir))
  from configurator import configure_file
  vars_dict = {
    '_user_name': '${_user_name}',
    '_host_name': '${_host_name}',
    '_fqdn': '${_fqdn}',
    '_processor_name': '${_processor_name}',
    '_processor_description': '${_processor_description}',
    '_os_name': '${_os_name}',
    '_os_release': '${_os_release}',
    '_os_version': '${_os_version}',
    '_os_platform': '${_os_platform}',
    '_configuration_time': '${_configuration_time}',
    'CMAKE_VERSION': '${CMAKE_VERSION}',
    'CMAKE_GENERATOR': '${CMAKE_GENERATOR}',
    'CMAKE_Fortran_COMPILER': '${CMAKE_Fortran_COMPILER}',
    'CMAKE_C_COMPILER': '${CMAKE_C_COMPILER}',
  }
  configure_file(input_file, output_file, vars_dict)
  ")

使用find_package让CMake使用Python解释器:

.. code-block:: cmake

  find_package(PythonInterp QUIET REQUIRED)

如果找到Python解释器，则可以在CMake中执行_config_script，并生成print_info.c文件:

.. code-block:: cmake

  execute_process(
    COMMAND
        ${PYTHON_EXECUTABLE} "-c" ${_config_script}
    )

之后，定义可执行目标和依赖项，这与前一个示例相同。所以，得到的输出没有变化。

**工作原理**

回顾一下对CMakeLists.txt的更改。

我们执行了一个Python脚本生成print_info.c。运行Python脚本前，首先检测Python解释器，并构造Python脚本。Python脚本导入configure_file函数，
我们在configuration.py中定义了这个函数。为它提供用于读写的文件位置，并将其值作为键值对。

此示例展示了生成配置的另一种方法，将生成任务委托给外部脚本，可以将配置报告编译成可执行文件，甚至库目标。我们在前面的配置中认为的第一种方法更简洁，
但是使用本示例中提供的方法，我们可以灵活地使用Python(或其他语言)，实现任何在配置时间所需的步骤。使用当前方法，
我们可以通过脚本的方式执行类似cmake_host_system_information()的操作。

但要记住，这种方法也有其局限性，它不能在构建时重新生成print_info.c的自动依赖项。下一个示例中，我们应对这个挑战


6.3 构建时使用Python生成源码
------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-6/recipe-03 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。


构建时根据某些规则生成冗长和重复的代码，同时避免在源代码存储库中显式地跟踪生成的代码生成源代码，是开发人员工具箱中的一个重要工具，
例如：根据检测到的平台或体系结构生成不同的源代码。或者，可以使用Python，根据配置时收集的输入，在构建时生成高效的C++代码。
其他生成器解析器，比如：Flex (https://github.com/westes/flex )和Bison(https://www.gnu.org/software/bison/ )；
元对象编译器，如Qt的moc(http://doc.qt.io/qt5/moc.html )；序列化框架，如谷歌的protobuf 
(https://developers.google.com/protocol-buffers/ )。

**准备工作**

为了提供一个具体的例子，我们需要编写代码来验证一个数字是否是质数。现在有很多算法，例如：可以用埃拉托色尼的筛子(sieve of Eratosthenes)
来分离质数和非质数。如果有很多验证数字，我们不希望对每一个数字都进行Eratosthenes筛选。我们想要做的是将所有质数一次制表，直到数字的上限，
然后使用一个表查的方式，找来验证大量的数字。

本例中，将在编译时使用Python为查找表(质数向量)生成C++代码。当然，为了解决这个特殊的编程问题，我们还可以使用C++生成查询表，并且可以在运行时执行查询。

让我们从generate.py脚本开始。这个脚本接受两个命令行参数——一个整数范围和一个输出文件名:

.. code-block:: python

  """
  Generates C++ vector of prime numbers up to max_number
  using sieve of Eratosthenes.
  """
  import pathlib
  import sys
  # for simplicity we do not verify argument list
  max_number = int(sys.argv[-2])
  output_file_name = pathlib.Path(sys.argv[-1])
  numbers = range(2, max_number + 1)
  is_prime = {number: True for number in numbers}
  for number in numbers:
    current_position = number
    if is_prime[current_position]:
      while current_position <= max_number:
        current_position += number
        is_prime[current_position] = False
  primes = (number for number in numbers if is_prime[number])
  code = """#pragma once
  #include <vector>
  const std::size_t max_number = {max_number};
  std::vector<int> & primes() {{
    static std::vector<int> primes;
    {push_back}
    return primes;
  }}
  """
  push_back = '\n'.join([' primes.push_back({:d});'.format(x) for x in primes])
  output_file_name.write_text(
  code.format(max_number=max_number, push_back=push_back))

我们的目标是生成一个primes.hpp，并将其包含在下面的示例代码中:

.. code-block:: c++

  #include "primes.hpp"
  #include <iostream>
  #include <vector>
  int main() {
    std::cout << "all prime numbers up to " << max_number << ":";
    for (auto prime : primes())
        std::cout << " " << prime;
    std::cout << std::endl;
    return 0;
  }

**具体实施**

下面是CMakeLists.txt命令的详解:

1 首先，定义项目并检测Python解释器:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-03 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(PythonInterp QUIET REQUIRED)

2 将生成的代码放在${CMAKE_CURRENT_BINARY_DIR}/generate下，需要告诉CMake创建这个目录:

.. code-block:: cmake

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)

3 Python脚本要求质数的上限，使用下面的命令，我们可以设置一个默认值:

.. code-block:: cmake

  set(MAX_NUMBER "100" CACHE STRING "Upper bound for primes")

4 接下来，定义一个自定义命令来生成头文件:

.. code-block:: cmake

  add_custom_command(
    OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/generated/primes.hpp
    COMMAND
        ${PYTHON_EXECUTABLE} generate.py ${MAX_NUMBER}     ${CMAKE_CURRENT_BINARY_DIR}/generated/primes.hpp
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS
        generate.py
  )

5 最后，定义可执行文件及其目标，包括目录和依赖关系:

.. code-block:: cmake

  add_executable(example "")
  target_sources(example
    PRIVATE
        example.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/generated/primes.hpp
    )
  target_include_directories(example
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/generated
  )

6 准备测试:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./example
  all prime numbers up to 100: 2 3 5 7 11 13 17 19 23 29 31 37 41 43 47 53 59 61 67 71 73 79

**具体实施**

为了生成头文件，我们定义了一个自定义命令，它执行generate.py脚本，并接受${MAX_NUMBER}和文件路径
(${CMAKE_CURRENT_BINARY_DIR}/generated/primes.hpp)作为参数:

.. code-block:: cmake

  add_custom_command(
    OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/generated/primes.hpp
    COMMAND
        ${PYTHON_EXECUTABLE} generate.py ${MAX_NUMBER} ${CMAKE_CURRENT_BINARY_DIR}/generated/primes.hpp
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS
        generate.py
    )

为了生成源代码，我们需要在可执行文件的定义中，使用target_sources很容易实现添加源代码作为依赖项:

.. code-block:: cmake

  target_sources(example
    PRIVATE
        example.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/generated/primes.hpp
    )

前面的代码中，我们不需要定义新的目标。头文件将作为示例的依赖项生成，并在每次generate.py脚本更改时重新生成。如果代码生成脚本生成多个源文件，
那么要将所有生成的文件列出，做为某些目标的依赖项。

**更多信息**

我们提到所有的生成文件，都应该作为某个目标的依赖项。但是，我们可能不知道这个文件列表，因为它是由生成文件的脚本决定的，这取决于我们提供给配置的输入。
这种情况下，我们可能会尝试使用file(GLOB…)将生成的文件收集到一个列表中(参见https://cmake.org/cmake/help/v3.5/command/file.html )。

file(GLOB…)在配置时执行，而代码生成是在构建时发生的。因此可能需要一个间接操作，将file(GLOB…)命令放在一个单独的CMake脚本中，
使用${CMAKE_COMMAND} -P执行该脚本，以便在构建时获得生成的文件列表。


6.4 记录项目版本信息以便报告
------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-6/recipe-04 中找到，其中包含一个C和Fortran例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

代码版本很重要，不仅是为了可重复性，还为了记录API功能或简化支持请求和bug报告。源代码通常处于某种版本控制之下，
例如：可以使用Git标记附加额外版本号(参见https://semver.org )。然而，不仅需要对源代码进行版本控制，而且可执行文件还需要记录项目版本，
以便将其打印到代码输出或用户界面上。

本例中，将在CMake源文件中定义版本号。我们的目标是在配置项目时将程序版本记录到头文件中。然后，生成的头文件可以包含在代码的正确位置和时间，
以便将代码版本打印到输出文件或屏幕上。

**准备工作**

将使用以下C文件(example.c)打印版本信息:

.. code-block:: c++

  #include "version.h"
  #include <stdio.h>
  int main() {
    printf("This is output from code %s\n", PROJECT_VERSION);
    printf("Major version number: %i\n", PROJECT_VERSION_MAJOR);
    printf("Minor version number: %i\n", PROJECT_VERSION_MINOR);
    printf("Hello CMake world!\n");
  }

这里，假设PROJECT_VERSION_MAJOR、PROJECT_VERSION_MINOR和PROJECT_VERSION是在version.h中定义的。目标是从以下模板中生成version.h.in:

.. code-block:: cmake

  #pragma once
  #define PROJECT_VERSION_MAJOR @PROJECT_VERSION_MAJOR@
  #define PROJECT_VERSION_MINOR @PROJECT_VERSION_MINOR@
  #define PROJECT_VERSION_PATCH @PROJECT_VERSION_PATCH@
  #define PROJECT_VERSION "v@PROJECT_VERSION@"

这里使用预处理器定义，也可以使用字符串或整数常量来提高类型安全性(稍后我们将对此进行演示)。从CMake的角度来看，这两种方法是相同的。

**如何实施**

我们将按照以下步骤，在模板头文件中对版本进行注册:

1 要跟踪代码版本，我们可以在CMakeLists.txt中调用CMake的project时定义项目版本:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-04 VERSION 2.0.1 LANGUAGES C)

2 然后，基于version.h.in生成version.h:

.. code-block:: cmake

  configure_file(
    version.h.in
    generated/version.h
    @ONLY
    )

3 最后，我们定义了可执行文件，并提供了目标包含路径:

.. code-block:: cmake

  add_executable(example example.c)
  target_include_directories(example
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/generated
    )

**工作原理**

当使用版本参数调用CMake的project时，CMake将为项目设置PROJECT_VERSION_MAJOR、PROJECT_VERSION_MINOR和PROJECT_VERSION_PATCH。
此示例中的关键命令是configure_file，它接受一个输入文件(本例中是version.h.in)，通过将@之间的占位符替换成对应的CMake变量，
生成一个输出文件(本例中是generate/version.h)。它将@PROJECT_VERSION_MAJOR@替换为2，以此类推。使用关键字@ONLY，
我们将configure_file限制为只替换@variables@，而不修改${variables}。后一种形式在version.h.in中没有使用。但是，当使用CMake配置shell脚本时，
会经常出现。

生成的头文件可以包含在示例代码中，可以打印版本信息:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./example
  This is output from code v2.0.1
  Major version number: 2
  Minor version number: 0
  Hello CMake world!

.. NOTE::

  CMake以x.y.z格式给出的版本号，并将变量PROJECT_VERSION和<project-name>_VERSION设置为给定的值。此外,
  PROJECT_VERSION_MAJOR(<project-name>_VERSION_MAJOR),PROJECT_VERSION_MINOR(<project-name>_VERSION_MINOR) 
  PROJECT_VERSION_PATCH(<project-name>_VERSION_PATCH)和PROJECT_VERSION_TWEAK(<project-name>_VERSION_TWEAK),
  将分别设置为X, Y, Z和t。

6.5 从文件中记录项目版本
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-6/recipe-05 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

这个示例的目的和前一个相似，但是出发点不同。我们计划是从文件中读取版本信息，而不是将其设置在CMakeLists.txt中。将版本保存在单独文件中的动机，
是允许其他构建框架或开发工具使用独立于CMake的信息，而无需将信息复制到多个文件中。与CMake并行使用的构建框架的一个例子是Sphinx文档框架，
它生成文档并将其部署到阅读文档服务中，以便在线提供代码文档。

**准备工作**

我们将从一个名为VERSION的文件开始，其中包含以下内容:

2.0.1-rc-2
这一次，选择更安全的数据类型，并将PROGRAM_VERSION定义为version.hpp.in中的字符串常量:

.. code-block:: bash

  #pragma once
  #include <string>
  const std::string PROGRAM_VERSION = "@PROGRAM_VERSION@";
  下面的源码(example.cpp)，将包含生成的version.hpp:

  // provides PROGRAM_VERSION
  #include "version.hpp"
  #include <iostream>
  int main() {
    std::cout << "This is output from code v" << PROGRAM_VERSION
    << std::endl;
    std::cout << "Hello CMake world!" << std::endl;
  }

**具体实施**

逐步来完成我们的任务:

1 CMakeLists.txt定义了最低版本、项目名称、语言和标准:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-05 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 从文件中读取版本信息如下:

.. code-block:: cmake

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION")
      file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" PROGRAM_VERSION)
      string(STRIP "${PROGRAM_VERSION}" PROGRAM_VERSION)
  else()
      message(FATAL_ERROR "File ${CMAKE_CURRENT_SOURCE_DIR}/VERSION not found")
  endif()

3 配置头文件:

.. code-block:: cmake

  configure_file(
    version.hpp.in
    generated/version.hpp
    @ONLY
    )

4 最后，定义了可执行文件及其依赖关系:

.. code-block:: cmake

  add_executable(example example.cpp)
  target_include_directories(example
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/generated
    )

5 进行测试:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./example
  This is output from code v2.0.1-rc-2
  Hello CMake world!

**工作原理**

我们使用以下构造，从一个名为VERSION的文件中读取版本字符串:

.. code-block:: cmake

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" PROGRAM_VERSION)
    string(STRIP "${PROGRAM_VERSION}" PROGRAM_VERSION)
  else()
      message(FATAL_ERROR "File ${CMAKE_CURRENT_SOURCE_DIR}/VERSION not found")
  endif()

这里，首先检查该文件是否存在，如果不存在，则发出错误消息。如果存在，将内容读入PROGRAM_VERSION变量中，该变量会去掉尾部的空格。
当设置了变量PROGRAM_VERSION，就可以使用它来配置version.hpp.in，生成generated/version.hpp:

.. code-block:: cmake

  configure_file(
    version.hpp.in
    generated/version.hpp
    @ONLY
    )

6.6 配置时记录Git Hash值
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-6/recipe-06 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

大多数现代源代码存储库都使用Git作为版本控制系统进行跟踪，这可以归功于存储库托管平台GitHub的流行。因此，我们将在本示例中使用Git；
然而，实际中会根据具体的动机和实现，可以转化为其他版本控制系统。我们以Git为例，提交的Git Hash决定了源代码的状态。因此，为了标记可执行文件，
我们将尝试将Git Hash记录到可执行文件中，方法是将哈希字符串记录在一个头文件中，该头文件可以包含在代码中。

**准备工作**

我们需要两个源文件，类似于前面的示例。其中一个将配置记录的Hash(version.hpp.in)，详情如下:

.. code-block:: bash

  #pragma once
  #include <string>
  const std::string GIT_HASH = "@GIT_HASH@";

还需要一个示例源文件(example.cpp)，将Hash打印到屏幕上:

.. code-block:: bash

  #include "version.hpp"
  #include <iostream>
  int main() {
      std::cout << "This code has been configured from version " << GIT_HASH << std::endl;
  }

此示例还假定在Git存储库中至少有一个提交。因此，使用git init初始化这个示例，并使用git add <filename>，然后使用git commit创建提交，
以便获得一个有意义的示例。

**具体实施**

下面演示了从Git记录版本信息的步骤:

1 定义项目和支持语言:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-06 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 定义GIT_HASH变量:

.. code-block:: cmake

  # in case Git is not available, we default to "unknown"
  set(GIT_HASH "unknown")
  # find Git and if available set GIT_HASH variable
  find_package(Git QUIET)
  if(GIT_FOUND)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%h
      OUTPUT_VARIABLE GIT_HASH
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      WORKING_DIRECTORY
          ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
  message(STATUS "Git hash is ${GIT_HASH}")

3 CMakeLists.txt剩余的部分，类似于之前的示例:

.. code-block:: cmake

  # generate file version.hpp based on version.hpp.in
  configure_file(
    version.hpp.in
    generated/version.hpp
    @ONLY
    )
  # example code
  add_executable(example example.cpp)
  # needs to find the generated header file
  target_include_directories(example
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/generated
    )

4 验证输出(Hash不同):

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  $ ./example
  This code has been configured from version d58c64f

**工作原理**

使用find_package(Git QUIET)来检测系统上是否有可用的Git。如果有(GIT_FOUND为True)，运行一个Git命令:${GIT_EXECUTABLE} 
log -1 --pretty=format:%h。这个命令给出了当前提交Hash的简短版本。当然，这里我们可以灵活地运行Git命令。我们要求execute_process
命令将结果放入名为GIT_HASH的变量中，然后删除任何尾随的空格。使用ERROR_QUIET，如果Git命令由于某种原因失败，我们不会停止配置。

由于Git命令可能会失败(源代码已经分发到Git存储库之外)，或者Git在系统上不可用，我们希望为这个变量设置一个默认值，如下所示:

.. code-block:: cmake

  set(GIT_HASH "unknown")

此示例有一个问题，Git Hash是在配置时记录的，而不是在构建时记录。下一个示例中，我们将演示如何实现后一种方法。


6.7 构建时记录Git Hash值
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-6/recipe-07 中找到，其中包含一个C++例子。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

前面的示例中，在配置时记录了代码存储库(Git Hash)的状态。然而，前一种方法有一个令人不满意的地方，如果在配置代码之后更改分支或提交更改，
则源代码中包含的版本记录可能指向错误的Git Hash值。在这个示例中，我们将演示如何在构建时记录Git Hash(或者，执行其他操作)，
以确保每次构建代码时都运行这些操作，因为我们可能只配置一次，但是会构建多次。

**准备工作**

我们将使用与之前示例相同的version.hpp.in，只会对example.cpp文件进行修改，以确保它打印构建时Git提交Hash值:

.. code-block:: bash

  #include "version.hpp"
  #include <iostream>
  int main() {
      std::cout << "This code has been built from version " << GIT_HASH << std::endl;
  }

**具体实施**

将Git信息保存到version.hpp头文件在构建时需要进行以下操作:

1 把前一个示例的CMakeLists.txt中的大部分代码移到一个单独的文件中，并将该文件命名为git-hash.cmake:

.. code-block:: cmake

  # in case Git is not available, we default to "unknown"
  set(GIT_HASH "unknown")
  # find Git and if available set GIT_HASH variable
  find_package(Git QUIET)
  if(GIT_FOUND)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%h
      OUTPUT_VARIABLE GIT_HASH
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      )
  endif()
  message(STATUS "Git hash is ${GIT_HASH}")
  # generate file version.hpp based on version.hpp.in
  configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/version.hpp.in
    ${TARGET_DIR}/generated/version.hpp
    @ONLY
    )

2 CMakeLists.txt熟悉的部分:

.. code-block:: cmake

  # set minimum cmake version
  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  # project name and language
  project(recipe-07 LANGUAGES CXX)
  # require C++11
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # example code
  add_executable(example example.cpp)
  # needs to find the generated header file
  target_include_directories(example
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/generated
    )

3 CMakeLists.txt的剩余部分，记录了每次编译代码时的Git Hash:

.. code-block:: cmake

  add_custom_command(
  OUTPUT
      ${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp
  ALL
  COMMAND
      ${CMAKE_COMMAND} -D TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/git-hash.cmake
  WORKING_DIRECTORY
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  # rebuild version.hpp every time
  add_custom_target(
  get_git_hash
  ALL
  DEPENDS
      ${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp
  )
  # version.hpp has to be generated
  # before we start building example
  add_dependencies(example get_git_hash)

**工作原理**

示例中，在构建时执行CMake代码。为此，定义了一个自定义命令:

.. code-block:: cmake

  add_custom_command(
    OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp
    ALL
    COMMAND
        ${CMAKE_COMMAND} -D TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/git-hash.cmake
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

我们还定义了一个目标:

.. code-block:: cmake

  add_custom_target(
    get_git_hash
    ALL
    DEPENDS
        ${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp
    )

自定义命令调用CMake来执行git-hash.cmake脚本。这里使用CLI的-P开关，通过传入脚本的位置实现的。请注意，可以像往常一样使用CLI开关-D传递选项。
git-hash.cmake脚本生成${TARGET_DIR}/generated/version.hpp。自定义目标被添加到ALL目标中，并且依赖于自定义命令的输出。
换句话说，当构建默认目标时，我们确保自定义命令已经运行。此外，自定义命令将ALL目标作为输出。这样，我们就能确保每次都会生成version.hpp了。

**更多信息**

我们可以改进配置，以便在记录的Git Hash外，包含其他的信息。检测构建环境是否“污染”(即是否包含未提交的更改和未跟踪的文件)，或者“干净”。
可以使用git describe --abbrev=7 --long --always --dirty --tags检测这些信息。根据可重现性，甚至可以将Git的状态，完整输出记录到头文件中，
我们将这些功能作为课后习题留给读者自己完成。



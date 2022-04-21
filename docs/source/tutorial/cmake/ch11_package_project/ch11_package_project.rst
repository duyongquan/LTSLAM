.. highlight:: c++

.. default-domain:: cpp

==========================
第11章 打包项目
==========================


11.1 生成源代码和二进制包
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-11/recipe-01 中找到。
  该示例在CMake 3.6版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

如果代码是开源的，用户将能够下载项目的源代码，并使用完全定制的CMake脚本自行构建。当然，打包操作也可以使用脚本完成，
但是CPack提供了更简单和可移植的替代方案。本示例将指导您创建一些包:


* 源代码包：可以将源代码直接压缩成归档文件，进行发送。用户将不必为特定的版本控制系统操心。
* 二进制包：工具将新构建的目标以打包的方式到归档文件中。这个功能非常有用，但可能不够健壮，无法发布库和可执行程序。
* 平台原生的二进制安装：CPack能够以许多不同的格式生成二进制安装程序，因此可以将软件发布到不同的平台。我们将展示如何生成安装程序:

  * 基于Debian的GNU/Linux发行版的.deb格式： https://manpages.debian.org/unstable/dpkg-dev/deb.5.en.html
  * 基于Red Hat的GNU/Linux发行版的.rpm格式： http://rpm.org/
  * macOS包的.dmg格式: https://developer.apple.com/library/archive/documentation/CoreFoundation/Conceptual/CFBundles/BundleTypes/BundleTypes.html
  * Windows的NSIS格式: http://nsis.sourceforge.net/Main_Page

*准备工作*

我们将使用第10章第3节的示例，项目树由以下目录和文件组成:

.. code-block:: bash

  .
  ├── cmake
  │    ├── coffee.icns
  │    ├── Info.plist.in
  │    └── messageConfig.cmake.in
  ├── CMakeCPack.cmake
  ├── CMakeLists.txt
  ├── INSTALL.md
  ├── LICENSE
  ├── src
  │    ├── CMakeLists.txt
  │    ├── hello-world.cpp
  │    ├── Message.cpp
  │    └── Message.hpp
  └── tests
      ├── CMakeLists.txt
      └── use_target
          ├── CMakeLists.txt
          └── use_message.cpp

由于本示例的重点是使用CPack，所以不会讨论源码。我们只会在CMakeCPack.cmake中添加打包指令。此外，还添加了INSTALL.md和LICENSE文件：
打包要求需要包含安装说明和项目许可信息。

**具体实施**

让我们看看需要添加到这个项目中的打包指令。我们将在CMakeCPack.cmake中收集它们，并在在CMakeLists.txt的末尾包含这个模块include(cmakecpackage.cmake):

1 我们声明包的名称，与项目的名称相同，因此我们使用PROJECT_NAME的CMake变量:

.. code-block:: cmake

  set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")

2 声明包的供应商：

.. code-block:: cmake

  set(CPACK_PACKAGE_VENDOR "CMake Cookbook")

3 打包的源代码将包括一个描述文件。这是带有安装说明的纯文本文件:

.. code-block:: cmake

  set(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/INSTALL.md")

4 还添加了一个包的描述:

.. code-block:: cmake

  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "message: a small messaging library")

5 许可证文件也将包括在包中:

.. code-block:: cmake

  set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

6 从发布包中安装时，文件将放在/opt/recipe-01目录下:

.. code-block:: cmake

  set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/${PROJECT_NAME}")

7 CPack所需的主要、次要和补丁版本:

.. code-block:: cmake

  set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
  set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
  set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")

8 设置了在包装的时候需要忽略的文件列表和目录:

.. code-block:: cmake

  set(CPACK_SOURCE_IGNORE_FILES "${PROJECT_BINARY_DIR};/.git/;.gitignore")

9 列出了源代码归档的打包生成器——在我们的例子中是ZIP，用于生成.ZIP归档，TGZ用于.tar.gz归档:

.. code-block:: cmake

  set(CPACK_SOURCE_GENERATOR "ZIP;TGZ")

10 我们还列出了二进制存档生成器:

.. code-block:: cmake

  set(CPACK_GENERATOR "ZIP;TGZ")

11 现在也可声明平台原生二进制安装程序，从DEB和RPM包生成器开始，不过只适用于GNU/Linux:

.. code-block:: cmake

  if(UNIX)
    if(CMAKE_SYSTEM_NAME MATCHES Linux)
      list(APPEND CPACK_GENERATOR "DEB")
      set(CPACK_DEBIAN_PACKAGE_MAINTAINER "robertodr")
      set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
      set(CPACK_DEBIAN_PACKAGE_DEPENDS "uuid-dev")
      list(APPEND CPACK_GENERATOR "RPM")
      set(CPACK_RPM_PACKAGE_RELEASE "1")
      set(CPACK_RPM_PACKAGE_LICENSE "MIT")
      set(CPACK_RPM_PACKAGE_REQUIRES "uuid-devel")
    endif()
  endif()

12 如果我们在Windows上，我们会想要生成一个NSIS安装程序:

.. code-block:: cmake

  if(WIN32 OR MINGW)
    list(APPEND CPACK_GENERATOR "NSIS")
    set(CPACK_NSIS_PACKAGE_NAME "message")
    set(CPACK_NSIS_CONTACT "robertdr")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
  endif()

13 另一方面，在macOS上，bundle包是我们的安装程序的选择:

.. code-block:: cmake

  if(APPLE)
    list(APPEND CPACK_GENERATOR "Bundle")
    set(CPACK_BUNDLE_NAME "message")
    configure_file(${PROJECT_SOURCE_DIR}/cmake/Info.plist.in Info.plist @ONLY)
    set(CPACK_BUNDLE_PLIST ${CMAKE_CURRENT_BINARY_DIR}/Info.plist)
    set(CPACK_BUNDLE_ICON ${PROJECT_SOURCE_DIR}/cmake/coffee.icns)
  endif()

14 我们在现有系统的包装生成器上，向用户打印一条信息:

.. code-block:: cmake

  message(STATUS "CPack generators: ${CPACK_GENERATOR}")

15 最后，我们包括了CPack.cmake标准模块。这将向构建系统添加一个包和一个package_source目标:

.. code-block:: cmake

  include(CPack)

现在来配置这个项目：

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..

使用下面的命令，我们可以列出可用的目标(示例输出是在使用Unix Makefile作为生成器的GNU/Linux系统上获得的):

.. code-block:: bash

  $ cmake --build . --target help
  The following are some of the valid targets for this Makefile:
  ... all (the default if no target is provided)
  ... clean
  ... depend
  ... install/strip
  ... install
  ... package_source
  ... package
  ... install/local
  ... test
  ... list_install_components
  ... edit_cache
  ... rebuild_cache
  ... hello- world
  ... message

我们可以看到package和package_source目标是可用的。可以使用以下命令生成源包:

.. code-block:: bash

  $ cmake --build . --target package_source
  Run CPack packaging tool for source...
  CPack: Create package using ZIP
  CPack: Install projects
  CPack: - Install directory: /home/user/cmake-cookbook/chapter-11/recipe-01/cxx-example
  CPack: Create package
  CPack: - package: /home/user/cmake-cookbook/chapter- 11/recipe-01/cxx-example/build/recipe-01-1.0.0-Source.zip generated.
  CPack: Create package using TGZ
  CPack: Install projects
  CPack: - Install directory: /home/user/cmake-cookbook/chapter- 11/recipe-01/cxx-example
  CPack: Create package
  CPack: - package: /home/user/cmake-cookbook/chapter-11/recipe-01/cxx-example/build/recipe-01- 1.0.0-Source.tar.gz generated.

同样，也可以构建二进制包:

.. code-block:: bash

  $ cmake --build . --target package message-1.0.0-Linux.deb

例子中，最后得到了以下二进制包:

.. code-block:: bash

  message-1.0.0-Linux.rpm
  message-1.0.0-Linux.tar.gz
  message-1.0.0-Linux.zip


11.2 通过PyPI发布使用CMake/pybind11构建的C++/Python项目
---------------------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-11/recipe-02 中找到。
  该示例在CMake 3.11版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

本示例中，我们将以第9章第5节的代码的pybind11为例，为其添加相关的安装目标和pip打包信息，并将项目上传到PyPI。我们要实现一个可以使用pip安装，
并运行CMake从而获取底层pybind11依赖项的项目。

**具体实施**

本示例基于第9章第5节项目的基础上。

1 首先，修改account/CMakeLists.txt，添加安装目标：

.. code-block:: cmake

  install(
    TARGETS
        account
    LIBRARY
        DESTINATION account
    )
    
安装目标时，README.rst, MANIFEST.in，setup.py、__init__.py和version.py将放置在对应的位置上，我们准备使用pybind11测试安装过程：

为此，在某处创建一个新目录，我们将在那里测试安装。

2 在创建的目录中，从本地路径运行pipenv install。调整本地路径，指向setup.py的目录：

.. code-block:: bash

  $ pipenv install /path/to/cxx-example

3 在Pipenv环境中打开一个Python shell：

.. code-block:: bash

  $ pipenv run python

4 Python shell中，可以测试我们的CMake包：

.. code-block:: bash

  >>> from account import Account
  >>> account1 = Account()
  >>> account1.deposit(100.0)
  >>> account1.deposit(100.0)
  >>> account1.withdraw(50.0)
  >>> print(account1.get_balance())
  150.0

11.3 通过PyPI发布使用CMake/CFFI构建C/Fortran/Python项目
---------------------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-11/recipe-03 中找到，其中有一个C++和Fortran示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

基于第9章第6节的示例，我们将重用前一个示例中的构建块，不过这次使用Python CFFI来提供Python接口，而不是pybind11。
这个示例中，我们通过PyPI共享一个Fortran项目，这个项目可以是C或C++项目，也可以是任何公开C接口的语言，非Fortran就可以。


**具体实施**

讨论一下实现打包的步骤：

1 示例基于第9章第6节，使用Python CFFI扩展了account/CMakeLists.txt，增加以下指令:

.. code-block:: cmake

  file(
    GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/interface_file_names.cfg
    INPUT ${CMAKE_CURRENT_SOURCE_DIR}/interface_file_names.cfg.in
    )
  set_target_properties(account
    PROPERTIES
      PUBLIC_HEADER "account.h;${CMAKE_CURRENT_BINARY_DIR}/account_export.h"
      RESOURCE "${CMAKE_CURRENT_BINARY_DIR}/interface_file_names.cfg"
    )
  install(
    TARGETS
      account
    LIBRARY
      DESTINATION account/lib
    RUNTIME
      DESTINATION account/lib
    PUBLIC_HEADER
      DESTINATION account/include
    RESOURCE
      DESTINATION account
    )

安装目标和附加文件准备好之后，就可以测试安装了。为此，会在某处创建一个新目录，我们将在那里测试安装。

2 新创建的目录中，我们从本地路径运行pipenv install。调整本地路径，指向setup.py脚本保存的目录:

.. code-block:: bash

  $ pipenv install /path/to/fortran-example

3 现在在Pipenv环境中生成一个Python shell:

.. code-block:: bash

  $ pipenv run python

4 Python shell中，可以测试CMake包:

.. code-block:: bash

  >>> import account
  >>> account1 = account.new()
  >>> account.deposit(account1, 100.0)
  >>> account.deposit(account1, 100.0)
  >>> account.withdraw(account1, 50.0)
  >>> print(account.get_balance(account1))
  150.0

11.4 以Conda包的形式发布一个简单的项目
---------------------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-11/recipe-04 中找到。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

虽然PyPI是发布Python包的标准平台，但Anaconda (https://anaconda.org )更为可能更为流行，因为它不仅允许使用Python接口发布Python或混合项目，
还允许对非Python项目进行打包和依赖关系管理。这个示例中，我们将为一个非常简单的C++示例项目准备一个Conda包，该项目使用CMake配置和构建，除了C++之外没有依赖关系。
下一个示例中，我们将会来看看一个更复杂的Conda包。

**具体实施**

1 CMakeLists.txt文件给出了最低版本要求、项目名称和支持的语言:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-04 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 使用example.cpp构建hello-conda可执行目标：

.. code-block:: cmake

  add_executable(hello-conda "")
  target_sources(hello-conda
    PRIVATE
        example.cpp
    )

3 使用CMakeLists.txt定义安装目标：

.. code-block:: cmake

  nstall(
    TARGETS
        hello-conda
    DESTINATION
        bin
    )

4 将在一个名为meta.yaml的文件中，对Conda包进行描述。我们将把它放在conda-recipe目录下，文件结构如下：

.. code-block:: bash

  .
  ├── CMakeLists.txt
  ├── conda-recipe
  │    └── meta.yaml
  └── example.cpp

5 meta.yaml包含如下内容：

.. code-block:: bash

  package:
    name: conda-example-simple
    version: "0.0.0"
  source:
    path: .. /  # this can be changed to git-url
  build:
    number: 0
    binary_relocation: true
    script:
      - cmake -H. -Bbuild_conda -G "${CMAKE_GENERATOR}" -DCMAKE_INSTALL_PREFIX=${PREFIX} # [not win]
      - cmake -H. -Bbuild_conda -G "%CMAKE_GENERATOR%" -DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" # [win]
      - cmake - -build build_conda - -target install
  requirements:
    build:
      - cmake >=3.5
      - { { compiler('cxx') } }
  about:
    home: http://www.example.com
    license: MIT
    summary: "Summary in here ..."

6 现在来构建包：

.. code-block:: bash

  $ conda build conda-recipe

7 过程中屏幕上看到大量输出，但是一旦构建完成，就可以对包进行安装。首先，在本地进行测试：

.. code-block:: bash

  $ conda install --use-local conda-example-simple

8 现在准备测试安装包，打开一个新的终端(假设Anaconda处于激活状态)，并输入以下内容：

.. code-block:: bash

  $ hello-conda
  hello from your conda package!

9 测试成功后，再移除包装：

.. code-block:: bash

  $ conda remove conda-example-simple


11.5 将Conda包作为依赖项发布给项目
---------------------------------------------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-11/recipe-05 中找到。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

这个示例中，我们将基于之前示例的结果，并且为CMake项目准备一个更真实和复杂的Conda包，这将取决于DGEMM的函数实现，对于矩阵与矩阵的乘法，
可以使用Intel的MKL库进行。Intel的MKL库可以以Conda包的形式提供。此示例将为我们提供一个工具集，用于准备和共享具有依赖关系的Conda包。

**具体实施**

1 CMakeLists.txt文件声明了最低版本、项目名称和支持的语言：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-05 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 使用example.cpp构建dgem-example可执行目标：

.. code-block:: cmake

  add_executable(dgemm-example "")
  target_sources(dgemm-example
    PRIVATE
        example.cpp
    )

3 然后，需要找到通过MKL-devel安装的MKL库。我们准备了一个名为IntelMKL的INTERFACE库，该库可以用于其他目标，并将为依赖的目标设置包括目录、
编译器选项和链接库。根据Intel的建议(https://software.intel.com/en-us/articles/intel-mml-link-line-advisor/ )进行设置。首先，设置编译器选项：

.. code-block:: cmake

  add_library(IntelMKL INTERFACE)
  target_compile_options(IntelMKL
    INTERFACE
        $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:AppleClang>>:-m64>
    )

4 接下来，查找mkl.h头文件，并为IntelMKL目标设置include目录：

.. code-block:: cmake

  find_path(_mkl_h
    NAMES
        mkl.h
    HINTS
        ${CMAKE_INSTALL_PREFIX}/include
    )
  target_include_directories(IntelMKL
    INTERFACE
        ${_mkl_h}
    )
  message(STATUS "MKL header file FOUND: ${_mkl_h}")

5 最后，为IntelMKL目标设置链接库:

.. code-block:: cmake

  find_library(_mkl_libs
    NAMES
      mkl_rt
    HINTS
      ${CMAKE_INSTALL_PREFIX}/lib
    )
  message(STATUS "MKL single dynamic library FOUND: ${_mkl_libs}")
  find_package(Threads QUIET)
  target_link_libraries(IntelMKL
    INTERFACE
      ${_mkl_libs}
      $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:AppleClang>>:Threads::Threads>
      $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:AppleClang>>:m>
    )

6 使用cmake_print_properties函数，打印IntelMKL目标的信息：

.. code-block:: cmake

  include(CMakePrintHelpers)
  cmake_print_properties(
    TARGETS
        IntelMKL
    PROPERTIES
      INTERFACE_COMPILE_OPTIONS
      INTERFACE_INCLUDE_DIRECTORIES
      INTERFACE_LINK_LIBRARIES
    )

7 将这些库连接到dgem-example:

.. code-block:: cmake

  target_link_libraries(dgemm-example
    PRIVATE
        IntelMKL
    )

8 CMakeLists.txt中定义了安装目标:

.. code-block:: cmake

  install(
    TARGETS
        dgemm-example
    DESTINATION
        bin
    )

9 尝试构建包：

.. code-block:: bash

  $ conda build conda-recipe

10 过程中屏幕上将看到大量输出，但是一旦构建完成，就可以对包进行安装包。首先，在本地进行安装测试：

.. code-block:: bash

  $ conda install --use-local conda-example-dgemm

11 现在测试安装，打开一个新的终端(假设Anaconda处于激活状态)，并输入：

.. code-block:: bash

  $ dgemm-example

  MKL DGEMM example worked!

12 安装成功之后，再进行卸载：

.. code-block:: bash

  $ conda remove conda-example-dgemm




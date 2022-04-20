.. highlight:: c++

.. default-domain:: cpp

==========================
第12章 构建文档
==========================

12.1 使用Doxygen构建文档
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-12/recipe-01 中找到，其中包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

Doxygen(http://www.doxygen.nl )是非常流行的源代码文档工具。可以在代码中添加文档标记作为注释，而后运行Doxygen提取这些注释，
并以Doxyfile配置文件中定义的格式创建文档。Doxygen可以输出HTML、XML，甚至LaTeX或PDF。本示例将展示，如何使用CMake来构建Doxygen文档。

**准备工作**

使用前几章中介绍的消息库的简化版本。目录结构如下:

.. code-block:: bash

  .
  ├── cmake
  │    └── UseDoxygenDoc.cmake
  ├── CMakeLists.txt
  ├── docs
  │    ├── Doxyfile.in
  │    └── front_page.md
  └── src
      ├── CMakeLists.txt
      ├── hello-world.cpp
      ├── Message.cpp
      └── Message.hpp

我们仍然在src子目录下放置源代码，并且在CMake子目录中有自定义的CMake模块。由于重点是文档，所以消除了对UUID的依赖，并简化了源代码。
最大的区别是头文件中的大量代码注释：

.. code-block:: bash

  #include <iosfwd>
  #include <string>
  / * ! \file Message.hpp * /
  /*! \class Message
  * \brief Forwards string to screen
  * \author Roberto Di Remigio
  * \date 2018
  * /
  class Message {
  public:
    /*! \brief Constructor from a string
    * \param[in] m a message
    */
    Message(const std::string &m) : message_(m) {}
    /*! \brief Constructor from a character array
    * \param[in] m a message
    */
    Message(const char * m): message_(std:: string(m)){}
    friend std::ostream &operator<<(std::ostream &os, Message &obj) {
      return obj.printObject(os);
    }
  private:
    /*! The message to be forwarded to screen */
    std::string message_;
    /*! \brief Function to forward message to screen
    * \param[in, out] os output stream
    */
    std::ostream &printObject(std::ostream &os);
  };

这些注释的格式是/*!*/，并包含一些Doxygen可以理解的特殊标记(参见http://www.stack.nl/~dimitri/Doxygen/manual/docblocks.html )。

**具体实施**

首先，来看看根目录下的CMakeLists.txt：

1 我们声明了一个C++11项目：

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
  project(recipe-01 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 为动态库和静态库，以及可执行文件定义了输出目录：

.. code-block:: cmake

  include(GNUInstallDirs)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

3 将cmake子目录追加到CMAKE_MODULE_PATH。这是需要CMake找到我们的自定义模块：

.. code-block:: cmake

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

4 UseDoxygenDoc.cmake自定义模块。将在后面讨论它的内容:

.. code-block:: cmake

  include(UseDoxygenDoc)

5 然后添加src子目录:

.. code-block:: cmake

  add_subdirectory(src)

src子目录中的CMakeLists.txt文件包含以下构建块:

1 添加了消息库:

.. code-block:: cmake

  add_library(message STATIC
    Message.hpp
    Message.cpp
    )

2 然后，声明add_doxygen_doc函数。这个函数可以理解这些参数：BUILD_DIR、DOXY_FILE、TARGET_NAME和COMMENT。
使用cmake_parse_arguments标准CMake命令解析这些参数：

.. code-block:: cmake

  function(add_doxygen_doc)
    set(options)
    set(oneValueArgs BUILD_DIR DOXY_FILE TARGET_NAME COMMENT)
    set(multiValueArgs)
    cmake_parse_arguments(DOXY_DOC
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
      ${ARGN}
    )
    # ...
  endfunction()

3 Doxyfile包含用于构建文档的所有Doxygen设置。一个模板Doxyfile.in文件作为函数参数DOXY_FILE传递，并解析为DOXY_DOC_DOXY_FILE变量。
使用如下方式，配置模板文件Doxyfile.in:

.. code-block:: cmake

  configure_file(
    ${DOXY_DOC_DOXY_FILE}
    ${DOXY_DOC_BUILD_DIR}/Doxyfile
    @ONLY
    )

4 然后，定义了一个名为DOXY_DOC_TARGET_NAME的自定义目标，它将使用Doxyfile中的设置执行Doxygen，并在DOXY_DOC_BUILD_DIR中输出结果:

.. code-block:: cmake

  add_custom_target(${DOXY_DOC_TARGET_NAME}
    COMMAND
      ${DOXYGEN_EXECUTABLE} Doxyfile
    WORKING_DIRECTORY
      ${DOXY_DOC_BUILD_DIR}
    COMMENT
      "Building ${DOXY_DOC_COMMENT} with Doxygen"
    VERBATIM
    )

5 最后，为用户打印一条状态信息:

.. code-block:: cmake

  message(STATUS "Added ${DOXY_DOC_TARGET_NAME} [Doxygen] target to build documentation")

可以像往常一样配置项目：

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .

可以通过调用自定义文档目标来构建文档:

.. code-block:: bash

  $ cmake --build . --target docs

您将注意到构建树中出现了一个_build子目录。它包含Doxygen从源文件生成的HTML文档。用浏览器打开index.html将显示Doxygen欢迎页面。

如果导航到类列表，例如：可以浏览Message类的文档:


12.2 使用Sphinx构建文档
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-12/recipe-02 中找到，其中包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

Sphinx是一个Python程序，也是一个非常流行的文档系统(http://www.sphinx-doc.org )。当与Python项目一起使用时，可以为docstring解析源文件，
并自动为函数和类生成文档页面。然而，Sphinx不仅限于Python，还可以解析reStructuredText、Markdown，并生成HTML、ePUB或PDF文档。
还有在线阅读服务(https://readthedocs.org )，它提供了一种快速编写和部署文档的方法。本示例将向您展示，如何使用CMake构建Sphinx文档。

**准备工作**

我们希望建立一个简单的网站，记录我们的消息库输出的信息。源码树现在看起来如下:

.. code-block:: bash

  .
  ├── cmake
  │    ├── FindSphinx.cmake
  │    └── UseSphinxDoc.cmake
  ├── CMakeLists.txt
  ├── docs
  │    ├── conf.py.in
  │    └── index.rst
  └── src
      ├── CMakeLists.txt
      ├── hello-world.cpp
      ├── Message.cpp
      s└── Message.hpp

cmake子目录中有一些自定义模块，docs子目录以纯文本reStructuredText格式的网站主页，index.rst和一个带有Sphinx的设置Python模板文件conf.py.in，
这个模板文件可以使用sphinx-quickstart程序自动生成。

**具体实施**

与之前的示例相比，我们将修改主CMakeLists.txt文件，并实现一个函数(add_sphinx_doc):

1 将cmake文件夹附加到CMAKE_MODULE_PATH之后，我们将包括UseSphinxDoc.cmake自定义模块:

.. code-block:: cmake

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  include(UseSphinxDoc)

2 UseSphinxDoc.cmake模块定义了add_sphinx_doc函数。我们使用关键字参数调用这个函数，以便对Sphinx文档的构建进行设置。自定义文档目标将称为docs:

.. code-block:: cmake

  add_sphinx_doc(
    SOURCE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/docs
    BUILD_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/_build
    CACHE_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/_doctrees
    HTML_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/sphinx_html
    CONF_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/docs/conf.py.in
    TARGET_NAME
        docs
    COMMENT
        "HTML documentation"
    )

UseSphinxDoc.cmake模块遵循相同的显式方式，这样的使用方式要优于在前一个示例中的隐式方式：

1 需要找到Python解释器和Sphinx可执行文件，如下:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)
  find_package(Sphinx REQUIRED)

2 然后，用一个值关键字参数定义add_sphinx_doc函数，并用cmake_parse_arguments解析：

.. code-block:: cmake

  function(add_sphinx_doc)
    set(options)
    set(oneValueArgs
      SOURCE_DIR
      BUILD_DIR
      CACHE_DIR
      HTML_DIR
      CONF_FILE
      TARGET_NAME
      COMMENT
      )
    set(multiValueArgs)
    cmake_parse_arguments(SPHINX_DOC
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
      ${ARGN}
      )
    # ...
  endfunction()

3 模板文件conf.py.in作为CONF_FILE关键字参数传递，在SPHINX_DOC_BUILD_DIR中配置为conf.py：

.. code-block:: cmake

  configure_file(
    ${SPHINX_DOC_CONF_FILE}
    ${SPHINX_DOC_BUILD_DIR}/conf.py
    @ONLY
    )

4 添加了一个名为SPHINX_DOC_TARGET_NAME的自定义目标，用Sphinx来编排文档构建:

.. code-block:: cmake

  add_custom_target(${SPHINX_DOC_TARGET_NAME}
    COMMAND
      ${SPHINX_EXECUTABLE}
        -q
        -b html
        -c ${SPHINX_DOC_BUILD_DIR}
        -d ${SPHINX_DOC_CACHE_DIR}
        ${SPHINX_DOC_SOURCE_DIR}
        ${SPHINX_DOC_HTML_DIR}
    COMMENT
      "Building ${SPHINX_DOC_COMMENT} with Sphinx"
    VERBATIM
    )

5 最后，打印一条状态信息:

.. code-block:: cmake

  message(STATUS "Added ${SPHINX_DOC_TARGET_NAME} [Sphinx] target to build documentation")

配置项目并构建了文档目标:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build . --target docs

这将生成SPHINX_DOC_HTML_DIR中的HTML文档生成树的子目录。同样，可以使用浏览器打开index.html，并查看文档:

12.3 结合Doxygen和Sphinx
--------------------------

.. NOTE::

  此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-12/recipe-03 中找到，其中包含一个C++示例。
  该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

我们有一个C++项目，因此Doxygen是生成源代码文档的理想选择。然而，我们也希望发布面向用户的文档，例如：介绍设计选择。所以我们想使用Sphinx，
因为生成的HTML也可以在移动设备上查看，而且可以部署文档进行在线阅读(https://readthedocs.org )。
本教程将演示如何使用Breathe插件(https://breathe.readthedocs.io )组合Doxygen和Sphinx。

**准备工作**

这个示例的目录结构，类似于之前的两个示例:

.. code-block:: bash

  .
  ├── cmake
  │    ├── FindPythonModule.cmake
  │    ├── FindSphinx.cmake
  │    └── UseBreathe.cmake
  ├── CMakeLists.txt
  ├── docs
  │    ├── code-reference
  │    │    ├── classes-and-functions.rst
  │    │    └── message.rst
  │    ├── conf.py.in
  │    ├── Doxyfile.in
  │    └── index.rst
  └── src
      ├── CMakeLists.txt
      ├── hello-world.cpp
      ├── Message.cpp
      └── Message.hpp

docs子目录现在同时包含一个Doxyfile.in和一个conf.py.in模板文件。模板文件中，分别设置了Doxygen和Sphinx。此外，还有一个code-referenc子目录。

code-referenc子目录中的文件包含Breathe指令，用来在Sphinx中包含doxygen生成的文档：

.. code-block:: bash

  Messaging classes
  =================
  Message
  -------
  ..  doxygenclass:: Message
      :project: recipe-03
      :members:
      :protected-members:
      :private-members:

这将输出Message类的文档。

**具体实施**

src目录中的CMakeLists.txt文件没有改变。主CMakeLists.txt文件中有修改：

1 包含UseBreathe.cmake自定义模块：

.. code-block:: cmake

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  include(UseBreathe)

2 调用add_breathe_doc函数，这个函数是在自定义模块中定义的，它接受关键字参数，来设置Doxygen和Sphinx：

.. code-block:: cmake

  add_breathe_doc(
    SOURCE_DIR
      ${CMAKE_CURRENT_SOURCE_DIR}/docs
    BUILD_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/_build
    CACHE_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/_doctrees
    HTML_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/html
    DOXY_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in
    CONF_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/docs/conf.py.in
    TARGET_NAME
      docs
    COMMENT
      "HTML documentation"
    )

让我们看一下UseBreatheDoc.cmake模块，其遵循了与我们在前两个示例中描述的显式模式。具体描述如下:

1 文档生成依赖于Doxygen:

.. code-block:: cmake

  find_package(Doxygen REQUIRED)
  find_package(Perl REQUIRED)

2 还依赖于Python解释器和Sphinx:

.. code-block:: cmake

  find_package(PythonInterp REQUIRED)
  find_package(Sphinx REQUIRED)

3 此外，还必须找到breathe的Python模块。这里，我们使用FindPythonModule.cmake模块:

.. code-block:: cmake

  include(FindPythonModule)
  find_python_module(breathe REQUIRED)

4 定义了add_breathe_doc函数，这个函数有一个单值关键字参数，我们将使用cmake_parse_arguments命令解析它:

.. code-block:: cmake

  function(add_breathe_doc)
    set(options)
    set(oneValueArgs
      SOURCE_DIR
      BUILD_DIR
      CACHE_DIR
      HTML_DIR
      DOXY_FILE
      CONF_FILE
      TARGET_NAME
      COMMENT
      )
    set(multiValueArgs)
    cmake_parse_arguments(BREATHE_DOC
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
      ${ARGN}
      )
    # ...
  endfunction()

5 BREATHE_DOC_CONF_FILE中的Sphinx模板文件，会通过conf.py配置到的BREATHE_DOC_BUILD_DIR目录下：

.. code-block:: cmake

  configure_file(
    ${BREATHE_DOC_CONF_FILE}
    ${BREATHE_DOC_BUILD_DIR}/conf.py
    @ONLY
    )

6 相应地，Doxygen的BREATHE_DOC_DOXY_FILE模板文件配置为BREATHE_DOC_BUILD_DIR中的Doxyfile:

.. code-block:: cmake

  configure_file(
    ${BREATHE_DOC_DOXY_FILE}
    ${BREATHE_DOC_BUILD_DIR}/Doxyfile
    @ONLY
    )

7 添加BREATHE_DOC_TARGET_NAME自定义目标。注意，只有Sphinx在运行时，对Doxygen的调用才发生在BREATHE_DOC_SPHINX_FILE中:

.. code-block:: cmake

  add_custom_target(${BREATHE_DOC_TARGET_NAME}
    COMMAND
      ${SPHINX_EXECUTABLE}
        -q
        -b html
        -c ${BREATHE_DOC_BUILD_DIR}
        -d ${BREATHE_DOC_CACHE_DIR}
        ${BREATHE_DOC_SOURCE_DIR}
        ${BREATHE_DOC_HTML_DIR}
    COMMENT
      "Building ${BREATHE_DOC_TARGET_NAME} documentation with Breathe, Sphinx and Doxygen"
    VERBATIM
    )

8 最后，打印一条状态信息:

.. code-block:: cmake

  message(STATUS "Added ${BREATHE_DOC_TARGET_NAME} [Breathe+Sphinx+Doxygen] target to build documentation")

配置完成后，构建文档:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build . --target docs
  
该文档将在BREATHE_DOC_HTML_DIR子目录中可用。启动浏览器打开index.html文件后，可以导航到Message类的文档:




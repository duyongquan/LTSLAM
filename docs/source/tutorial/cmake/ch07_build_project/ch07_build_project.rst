.. highlight:: c++

.. default-domain:: cpp

==========================
第7章 构建项目
==========================

7.1 使用函数和宏重用代码
--------------------------

.. NOTE::
    
    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-01 中找到，其中包含一个C++例子。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

任何编程语言中，函数允许我们抽象(隐藏)细节并避免代码重复，CMake也不例外。本示例中，我们将以宏和函数为例进行讨论，并介绍一个宏，
以便方便地定义测试和设置测试的顺序。我们的目标是定义一个宏，能够替换add_test和set_tests_properties，用于定义每组和设置每个测试的预期开销(第4章，第8节)。

**准备工作**

我们将基于第4章第2节中的例子。main.cpp、sum_integers.cpp和sum_integers.hpp文件不变，用来计算命令行参数提供的整数队列的和。
单元测试(test.cpp)的源代码也没有改变。我们还需要Catch 2头文件，catch.hpp。与第4章相反，我们将把源文件放到子目录中，并形成以下文件树
(稍后我们将讨论CMake代码):

.. code-block:: bash
    .
    ├── CMakeLists.txt
    ├── src
    │     ├── CMakeLists.txt
    │     ├── main.cpp
    │     ├── sum_integers.cpp
    │     └── sum_integers.hpp
    └── tests
        ├── catch.hpp
        ├── CMakeLists.txt
        └── test.cpp

**具体实施**

1 定义了CMake最低版本、项目名称和支持的语言，并要求支持C++11标准:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-01 LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

2 根据GNU标准定义binary和library路径:

.. code-block:: cmake

    include(GNUInstallDirs)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

3 最后，使用add_subdirectory调用src/CMakeLists.txt和tests/CMakeLists.txt:

.. code-block:: cmake

    add_subdirectory(src)
    enable_testing()
    add_subdirectory(tests)

4 src/CMakeLists.txt定义了源码目标:

.. code-block:: cmake

    set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)
    add_library(sum_integers sum_integers.cpp)
    add_executable(sum_up main.cpp)
    target_link_libraries(sum_up sum_integers)

5 tests/CMakeLists.txt中，构建并链接cpp_test可执行文件:

.. code-block:: cmake

    add_executable(cpp_test test.cpp)
    target_link_libraries(cpp_test sum_integers)

6 定义一个新宏add_catch_test:

.. code-block:: cmake

    macro(add_catch_test _name _cost)
    math(EXPR num_macro_calls "${num_macro_calls} + 1")
    message(STATUS "add_catch_test called with ${ARGC} arguments: ${ARGV}")
    set(_argn "${ARGN}")
    if(_argn)
        message(STATUS "oops - macro received argument(s) we did not expect: ${ARGN}")
    endif()
    add_test(
        NAME
        ${_name}
        COMMAND
        $<TARGET_FILE:cpp_test>
        [${_name}] --success --out
        ${PROJECT_BINARY_DIR}/tests/${_name}.log --durations yes
        WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
        )
    set_tests_properties(
        ${_name}
        PROPERTIES
            COST ${_cost}
        )
    endmacro()

7 最后，使用add_catch_test定义了两个测试。此外，还设置和打印了变量的值:

.. code-block:: cmake

    set(num_macro_calls 0)
    add_catch_test(short 1.5)
    add_catch_test(long 2.5 extra_argument)
    message(STATUS "in total there were ${num_macro_calls} calls to add_catch_test")

8 现在，进行测试。配置项目(输出行如下所示):

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..
    -- ...
    -- add_catch_test called with 2 arguments: short;1.5
    -- add_catch_test called with 3 arguments: long;2.5;extra_argument
    -- oops - macro received argument(s) we did not expect: extra_argument
    -- in total there were 2 calls to add_catch_test
    -- ...

9 最后，构建并运行测试:

.. code-block:: cmake

    $ cmake --build .
    $ ctest

10 长时间的测试会先开始:

.. code-block:: cmake

    Start 2: long
    1/2 Test #2: long ............................. Passed 0.00 sec
    Start 1: short
    2/2 Test #1: short ............................ Passed 0.00 sec
    100% tests passed, 0 tests failed out of 2

**工作原理**

这个配置中的新添加了add_catch_test宏。这个宏需要两个参数_name和_cost，可以在宏中使用这些参数来调用add_test和set_tests_properties。
参数前面的下划线，是为了向读者表明这些参数只能在宏中访问。另外，宏自动填充了${ARGC}(参数数量)和${ARGV}(参数列表)，我们可以在输出中
验证了这一点:

.. code-block:: cmake

    -- add_catch_test called with 2 arguments: short;1.5
    -- add_catch_test called with 3 arguments: long;2.5;extra_argument

宏还定义了${ARGN}，用于保存最后一个参数之后的参数列表。此外，我们还可以使用${ARGV0}、${ARGV1}等来处理参数。我们演示一下，
如何捕捉到调用中的额外参数(extra_argument):

.. code-block:: cmake

    add_catch_test(long 2.5 extra_argument)

我们使用了以下方法:

.. code-block:: cmake

    set(_argn "${ARGN}")
    if(_argn)
        message(STATUS "oops - macro received argument(s) we did not expect: ${ARGN}")
    endif()

这个if语句中，我们引入一个新变量，但不能直接查询ARGN，因为它不是通常意义上的CMake变量。使用这个宏，
我们可以通过它们的名称和命令来定义测试，还可以指示预期的开销，这会让耗时长的测试在耗时短测试之前启动，这要归功于COST属性。

我们可以用一个函数来实现它，而不是使用相同语法的宏:

.. code-block:: cmake

    function(add_catch_test _name _cost)
        ...
    endfunction()

宏和函数之间的区别在于它们的变量范围。宏在调用者的范围内执行，而函数有自己的变量范围。换句话说，如果我们使用宏，
需要设置或修改对调用者可用的变量。如果不去设置或修改输出变量，最好使用函数。我们注意到，可以在函数中修改父作用域变量，
但这必须使用PARENT_SCOPE显式表示:

.. code-block:: cmake

    set(variable_visible_outside "some value" PARENT_SCOPE)

为了演示作用域，我们在定义宏之后编写了以下调用:

.. code-block:: cmake

    set(num_macro_calls 0)
    add_catch_test(short 1.5)
    add_catch_test(long 2.5 extra_argument)
    message(STATUS "in total there were ${num_macro_calls} calls to add_catch_test")

在宏内部，将num_macro_calls加1:

.. code-block:: cmake

    math(EXPR num_macro_calls "${num_macro_calls} + 1")

这时产生的输出:

    .. code-block:: cmake

    -- in total there were 2 calls to add_catch_test
    
如果我们将宏更改为函数，测试仍然可以工作，但是num_macro_calls在父范围内的所有调用中始终为0。
将CMake宏想象成类似函数是很有用的，这些函数被直接替换到它们被调用的地方(在C语言中内联)。将CMake函数想象成黑盒函数很有必要。
黑盒中，除非显式地将其定义为PARENT_SCOPE，否则不会返回任何内容。CMake中的函数没有返回值。

**更多信息**

可以在宏中嵌套函数调用，也可以在函数中嵌套宏调用，但是这就需要仔细考虑变量的作用范围。如果功能可以使用函数实现，
那么这可能比宏更好，因为它对父范围状态提供了更多的默认控制。

我们还应该提到在src/cmakelist .txt中使用CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE:

.. code-block:: cmake

    set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

这个命令会将当前目录，添加到CMakeLists.txt中定义的所有目标的interface_include_directory属性中。
换句话说，我们不需要使用target_include_directory来添加cpp_test所需头文件的位置。


7.2 将CMake源代码分成模块
-------------------------------

.. NOTE::
    
    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-02 中找到。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

项目通常从单个CMakeLists.txt文件开始，随着时间的推移，这个文件会逐渐增长。本示例中，我们将演示一种将CMakeLists.txt分割成更小单元的机制。
将CMakeLists.txt拆分为模块有几个动机，这些模块可以包含在主CMakeLists.txt或其他模块中:

* 主CMakeLists.txt更易于阅读。
* CMake模块可以在其他项目中重用。
* 与函数相结合，模块可以帮助我们限制变量的作用范围。

本示例中，我们将演示如何定义和包含一个宏，该宏允许我们获得CMake的彩色输出(用于重要的状态消息或警告)。

**准备工作**

本例中，我们将使用两个文件，主CMakeLists.txt和cmake/colors.cmake:

    .. code-block:: cmake
        
    .. code-block:: cmake

    .
    ├── cmake
    │     └── colors.cmake
    └── CMakeLists.txt

cmake/colors.cmake文件包含彩色输出的定义:

.. code-block:: cmake

    # colorize CMake output
    # code adapted from stackoverflow: http://stackoverflow.com/a/19578320
    # from post authored by https://stackoverflow.com/users/2556117/fraser
    macro(define_colors)
    if(WIN32)
        # has no effect on WIN32
        set(ColourReset "")
        set(ColourBold "")
        set(Red "")
        set(Green "")
        set(Yellow "")
        set(Blue "")
        set(Magenta "")
        set(Cyan "")
        set(White "")
        set(BoldRed "")
        set(BoldGreen "")
        set(BoldYellow "")
        set(BoldBlue "")
        set(BoldMagenta "")
        set(BoldCyan "")
        set(BoldWhite "")
    else()
        string(ASCII 27 Esc)
        set(ColourReset "${Esc}[m")
        set(ColourBold "${Esc}[1m")
        set(Red "${Esc}[31m")
        set(Green "${Esc}[32m")
        set(Yellow "${Esc}[33m")
        set(Blue "${Esc}[34m")
        set(Magenta "${Esc}[35m")
        set(Cyan "${Esc}[36m")
        set(White "${Esc}[37m")
        set(BoldRed "${Esc}[1;31m")
        set(BoldGreen "${Esc}[1;32m")
        set(BoldYellow "${Esc}[1;33m")
        set(BoldBlue "${Esc}[1;34m")
        set(BoldMagenta "${Esc}[1;35m")
        set(BoldCyan "${Esc}[1;36m")
        set(BoldWhite "${Esc}[1;37m")
    endif()
    endmacro()

**具体实施**

来看下我们如何使用颜色定义，来生成彩色状态消息:

1 从一个熟悉的头部开始:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-02 LANGUAGES NONE)

2 然后，将cmake子目录添加到CMake模块搜索的路径列表中:

.. code-block:: cmake

    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

3 包括colors.cmake模块，调用其中定义的宏:

.. code-block:: cmake

    include(colors)
    define_colors()

4 最后，打印了不同颜色的信息:

.. code-block:: cmake

    message(STATUS "This is a normal message")
    message(STATUS "${Red}This is a red${ColourReset}")
    message(STATUS "${BoldRed}This is a bold red${ColourReset}")
    message(STATUS "${Green}This is a green${ColourReset}")
    message(STATUS "${BoldMagenta}This is bold${ColourReset}")

5 测试一下(如果使用macOS或Linux，以下的输出应该出现屏幕上)

**工作原理**

这个例子中，不需要编译代码，也不需要语言支持，我们已经用LANGUAGES NONE明确了这一点：

.. code-block:: cmake

    project(recipe-02 LANGUAGES NONE)

我们定义了define_colors宏，并将其放在cmake/colors.cmake。因为还是希望使用调用宏中定义的变量，来更改消息中的颜色，
所以我们选择使用宏而不是函数。我们使用以下行包括宏和调用define_colors:

.. code-block:: cmake

    include(colors)
    define_colors()

我们还需要告诉CMake去哪里查找宏:

.. code-block:: cmake

    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(colors)命令指示CMake搜索${CMAKE_MODULE_PATH}，查找名称为colors.cmake的模块。

例子中，我们没有按以下的方式进行：

.. code-block:: cmake

    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    include(colors)

而是使用一个显式包含的方式:

.. code-block:: cmake

    include(cmake/colors.cmake)

7.3 编写函数来测试和设置编译器标志
-----------------------------------------------

.. NOTE::
    
    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-03 中找到，其中包含一个C/C++示例。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

前两个示例中，我们使用了宏。本示例中，将使用一个函数来抽象细节并避免代码重复。我们将实现一个接受编译器标志列表的函数。
该函数将尝试用这些标志逐个编译测试代码，并返回编译器理解的第一个标志。这样，我们将了解几个新特性：函数、列表操作、字符串操作，
以及检查编译器是否支持相应的标志。

**准备工作**

按照上一个示例的推荐，我们将在(set_compiler_flag.cmake)模块中定义函数，然后调用函数。该模块包含以下代码，我们将在后面详细讨论:

.. code-block:: cmake

    include(CheckCCompilerFlag)
    include(CheckCXXCompilerFlag)
    include(CheckFortranCompilerFlag)
    function(set_compiler_flag _result _lang)
    # build a list of flags from the arguments
    set(_list_of_flags)
    # also figure out whether the function
    # is required to find a flag
    set(_flag_is_required FALSE)
    foreach(_arg IN ITEMS ${ARGN})
        string(TOUPPER "${_arg}" _arg_uppercase)
        if(_arg_uppercase STREQUAL "REQUIRED")
            set(_flag_is_required TRUE)
        else()
            list(APPEND _list_of_flags "${_arg}")
        endif()
    endforeach()
    set(_flag_found FALSE)
    # loop over all flags, try to find the first which works
    foreach(flag IN ITEMS ${_list_of_flags})
        unset(_flag_works CACHE)
        if(_lang STREQUAL "C")
            check_c_compiler_flag("${flag}" _flag_works)
        elseif(_lang STREQUAL "CXX")
            check_cxx_compiler_flag("${flag}" _flag_works)
        elseif(_lang STREQUAL "Fortran")
            check_Fortran_compiler_flag("${flag}" _flag_works)
        else()
            message(FATAL_ERROR "Unknown language in set_compiler_flag: ${_lang}")
            endif()
        # if the flag works, use it, and exit
        # otherwise try next flag
        if(_flag_works)
        set(${_result} "${flag}" PARENT_SCOPE)
        set(_flag_found TRUE)
        break()
        endif()
    endforeach()
    # raise an error if no flag was found
    if(_flag_is_required AND NOT _flag_found)
        message(FATAL_ERROR "None of the required flags were supported")
    endif()
    endfunction()

**具体实施**

展示如何在CMakeLists.txt中使用set_compiler_flag函数:

1 定义最低CMake版本、项目名称和支持的语言(本例中是C和C++):

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-03 LANGUAGES C CXX)

2 显示包含set_compiler_flag.cmake:

.. code-block:: cmake

    include(set_compiler_flag.cmake)

3 测试C标志列表:

.. code-block:: cmake

    set_compiler_flag(
    working_compile_flag C REQUIRED
    "-foo" # this should fail
    "-wrong" # this should fail
    "-wrong" # this should fail
    "-Wall" # this should work with GNU
    "-warn all" # this should work with Intel
    "-Minform=inform" # this should work with PGI
    "-nope" # this should fail
    )
    message(STATUS "working C compile flag: ${working_compile_flag}")

4 测试C++标志列表:

.. code-block:: cmake

    set_compiler_flag(
    working_compile_flag CXX REQUIRED
    "-foo" # this should fail
    "-g" # this should work with GNU, Intel, PGI
    "/RTCcsu" # this should work with MSVC
    )
    message(STATUS "working CXX compile flag: ${working_compile_flag}")

5 现在，我们可以配置项目并验证输出。只显示相关的输出，相应的输出可能会因编译器的不同而有所不同:

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..
    -- ...
    -- Performing Test _flag_works
    -- Performing Test _flag_works - Failed
    -- Performing Test _flag_works
    -- Performing Test _flag_works - Failed
    -- Performing Test _flag_works
    -- Performing Test _flag_works - Failed
    -- Performing Test _flag_works
    -- Performing Test _flag_works - Success
    -- working C compile flag: -Wall
    -- Performing Test _flag_works
    -- Performing Test _flag_works - Failed
    -- Performing Test _flag_works
    -- Performing Test _flag_works - Success
    -- working CXX compile flag: -g
    -- ...

**工作原理**

这里使用的模式是:

定义一个函数或宏，并将其放入模块中
包含模块
调用函数或宏
从输出中，可以看到代码检查列表中的每个标志。一旦检查成功，它就打印成功的编译标志。
看看set_compiler_flag.cmake模块的内部，这个模块又包含三个模块:

.. code-block:: cmake

    include(CheckCCompilerFlag)
    include(CheckCXXCompilerFlag)
    include(CheckFortranCompilerFlag)

这都是标准的CMake模块，CMake将在${CMAKE_MODULE_PATH}中找到它们。这些模块分别提供check_c_compiler_flag、
check_cxx_compiler_flag和check_fortran_compiler_flag宏。然后定义函数:

.. code-block:: cmake

    function(set_compiler_flag _result _lang)
        ...
    endfunction()

set_compiler_flag函数需要两个参数，_result(保存成功编译标志或为空字符串)和_lang(指定语言:C、C++或Fortran)。

我们也能这样调用函数:

.. code-block:: cmake

    set_compiler_flag(working_compile_flag C REQUIRED "-Wall" "-warn all")

这里有五个调用参数，但是函数头只需要两个参数。这意味着REQUIRED、-Wall和-warn all将放在${ARGN}中。从${ARGN}开始，
我们首先使用foreach构建一个标志列表。同时，从标志列表中过滤出REQUIRED，并使用它来设置_flag_is_required:

.. code-block:: cmake

    # build a list of flags from the arguments
    set(_list_of_flags)
    # also figure out whether the function
    # is required to find a flag
    set(_flag_is_required FALSE)
    foreach(_arg IN ITEMS ${ARGN})
    string(TOUPPER "${_arg}" _arg_uppercase)
    if(_arg_uppercase STREQUAL "REQUIRED")
        set(_flag_is_required TRUE)
    else()
        list(APPEND _list_of_flags "${_arg}")
    endif()
    endforeach()

现在，我们将循环${_list_of_flags}，尝试每个标志，如果_flag_works被设置为TRUE，我们将_flag_found设置为TRUE，并中止进一步的搜索:

.. code-block:: cmake

    set(_flag_found FALSE)
    # loop over all flags, try to find the first which works
    foreach(flag IN ITEMS ${_list_of_flags})
    unset(_flag_works CACHE)
    if(_lang STREQUAL "C")
        check_c_compiler_flag("${flag}" _flag_works)
    elseif(_lang STREQUAL "CXX")
        check_cxx_compiler_flag("${flag}" _flag_works)
    elseif(_lang STREQUAL "Fortran")
        check_Fortran_compiler_flag("${flag}" _flag_works)
    else()
        message(FATAL_ERROR "Unknown language in set_compiler_flag: ${_lang}")
    endif()
    # if the flag works, use it, and exit
    # otherwise try next flag
    if(_flag_works)
        set(${_result} "${flag}" PARENT_SCOPE)
        set(_flag_found TRUE)
        break()
    endif()
    endforeach()

unset(_flag_works CACHE)确保check_*_compiler_flag的结果，不会在使用_flag_works result变量时，使用的是缓存结果。

如果找到了标志，并且_flag_works设置为TRUE，我们就将_result映射到的变量:

.. code-block:: cmake

    set(${_result} "${flag}" PARENT_SCOPE)

这需要使用PARENT_SCOPE来完成，因为我们正在修改一个变量，希望打印并在函数体外部使用该变量。
请注意，如何使用${_result}语法解引用，从父范围传递的变量_result的值。不管函数的名称是什么，这对于确保工作标志被设置非常有必要。
如果没有找到任何标志，并且该标志设置了REQUIRED，那我们将使用一条错误消息停止配置:

.. code-block:: cmake

    # raise an error if no flag was found
    if(_flag_is_required AND NOT _flag_found)
        message(FATAL_ERROR "None of the required flags were supported")
    endif()

7.4 用指定参数定义函数或宏
-------------------------------

.. NOTE::

    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-04 中找到，其中包含一个C++示例。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

前面的示例中，我们研究了函数和宏，并使用了位置参数。这个示例中，我们将定义一个带有命名参数的函数。我们将复用第1节中的示例，
使用函数和宏重用代码，而不是使用以下代码定义测试：add_catch_test(short 1.5)。

我们将这样调用函数:

.. code-block:: cmake

    add_catch_test(
        NAME
        short
    LABELS
        short
        cpp_test
    COST
        1.5
    )

**准备工作**

我们使用第1节中的示例，使用函数和宏重用代码，并保持C++源代码不变，文件树保持不变：

.. code-block:: cmake

    .
    ├── cmake
    │     └── testing.cmake
    ├── CMakeLists.txt
    ├── src
    │     ├── CMakeLists.txt
    │     ├── main.cpp
    │     ├── sum_integers.cpp
    │     └── sum_integers.hpp
    └── tests
        ├── catch.hpp
        ├── CMakeLists.txt
        └── test.cpp

**具体实施**

我们对CMake代码进行一些修改，如下所示:

1 CMakeLists.txt顶部中只增加了一行，因为我们将包括位于cmake下面的模块:

.. code-block:: cmake

    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

2 保持src/CMakeLists.txt。


3 tests/CMakeLists.txt中，将add_catch_test函数定义移动到cmake/testing.cmake，并且定义两个测试:

.. code-block:: cmake

    add_executable(cpp_test test.cpp)
    target_link_libraries(cpp_test sum_integers)
    include(testing)
    add_catch_test(
    NAME
        short
    LABELS
        short
        cpp_test
    COST
        1.5
    )
    add_catch_test(
    NAME
        long
    LABELS
        long
        cpp_test
    COST
        2.5
    )
    add_catch_test在cmake/testing.cmake中定义:

    function(add_catch_test)
    set(options)
    set(oneValueArgs NAME COST)
    set(multiValueArgs LABELS DEPENDS REFERENCE_FILES)
    cmake_parse_arguments(add_catch_test
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
        )
    message(STATUS "defining a test ...")
    message(STATUS " NAME: ${add_catch_test_NAME}")
    message(STATUS " LABELS: ${add_catch_test_LABELS}")
    message(STATUS " COST: ${add_catch_test_COST}")
    message(STATUS " REFERENCE_FILES: ${add_catch_test_REFERENCE_FILES}")
    add_test(
        NAME
            ${add_catch_test_NAME}
        COMMAND
            $<TARGET_FILE:cpp_test>
        [${add_catch_test_NAME}] --success --out
            ${PROJECT_BINARY_DIR}/tests/${add_catch_test_NAME}.log --durations yes
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
        )
    set_tests_properties(${add_catch_test_NAME}
        PROPERTIES
            LABELS "${add_catch_test_LABELS}"
        )
    if(add_catch_test_COST)
        set_tests_properties(${add_catch_test_NAME}
        PROPERTIES
            COST ${add_catch_test_COST}
        )
    endif()
    if(add_catch_test_DEPENDS)
        set_tests_properties(${add_catch_test_NAME}
        PROPERTIES
            DEPENDS ${add_catch_test_DEPENDS}
        )
    endif()
    if(add_catch_test_REFERENCE_FILES)
        file(
        COPY
            ${add_catch_test_REFERENCE_FILES}
        DESTINATION
            ${CMAKE_CURRENT_BINARY_DIR}
        )
    endif()
    endfunction()

4 测试输出:

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..
    -- ...
    -- defining a test ...
    -- NAME: short
    -- LABELS: short;cpp_test
    -- COST: 1.5
    -- REFERENCE_FILES:
    -- defining a test ...
    -- NAME: long
    -- LABELS: long;cpp_test
    -- COST: 2.5
    -- REFERENCE_FILES:
    -- ...

5 最后，编译并测试：

.. code-block:: cmake

    $ cmake --build .
    $ ctest

**工作原理**

示例的特点是其命名参数，因此我们可以将重点放在cmake/testing.cmake模块上。CMake提供cmake_parse_arguments命令，
我们使用函数名(add_catch_test)选项(我们的例子中是none)、单值参数(NAME和COST)和多值参数(LABELS、DEPENDS和REFERENCE_FILES)调用该命令:

.. code-block:: cmake

    function(add_catch_test)
    set(options)
    set(oneValueArgs NAME COST)
    set(multiValueArgs LABELS DEPENDS REFERENCE_FILES)
    cmake_parse_arguments(add_catch_test
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
        )
    ...
    endfunction()

cmake_parse_arguments命令解析选项和参数，并在例子中定义如下:

.. code-block:: cmake

    add_catch_test_NAME
    add_catch_test_COST
    add_catch_test_LABELS
    add_catch_test_DEPENDS
    add_catch_test_REFERENCE_FILES

可以查询，并在函数中使用这些变量。这种方法使我们有机会用更健壮的接口和更具有可读的函数/宏调用，来实现函数和宏。

**更多信息**

选项关键字(本例中我们没有使用)由cmake_parse_arguments定义为TRUE或FALSE。add_catch_test函数，还提供test命令作为一个命名参数，
为了更简洁的演示，我们省略了这个参数。

cmake_parse_arguments命令在cmake 3.5的版本前中的CMakeParseArguments.cmake定义。因此，可以在CMake/test.cmake
顶部的使用include(CMakeParseArguments)命令使此示例能与CMake早期版本一起工作。

7.5 重新定义函数和宏
--------------------------

.. NOTE::
    
    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-05 中找到。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

我们已经提到模块包含不应该用作函数调用，因为模块可能被包含多次。本示例中，我们将编写我们自己的“包含保护”机制，如果多次包含一个模块，
将触发警告。内置的include_guard命令从3.10版开始可以使用，对于C/C++头文件，它的行为就像#pragma一样。对于当前版本的CMake，
我们将演示如何重新定义函数和宏，并且展示如何检查CMake版本，对于低于3.10的版本，我们将使用定制的“包含保护”机制。

**准备工作**

这个例子中，我们将使用三个文件:

.. code-block:: cmake

    .
    ├── cmake
    │     ├── custom.cmake
    │     └── include_guard.cmake
    └── CMakeLists.txt

custom.cmake模块包含以下代码:

.. code-block:: cmake

    include_guard(GLOBAL)
    message(STATUS "custom.cmake is included and processed")

我们稍后会对cmake/include_guard.cmake进行讨论。

**具体实施**

我们对三个CMake文件的逐步分解:

1 示例中，我们不会编译任何代码，因此我们的语言要求是NONE:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-05 LANGUAGES NONE)

2 定义一个include_guard宏，将其放在一个单独的模块中:

.. code-block:: cmake

    # (re)defines include_guard
    include(cmake/include_guard.cmake)

3 cmake/include_guard.cmake文件包含以下内容(稍后将详细讨论):

.. code-block:: cmake

    macro(include_guard)
    if (CMAKE_VERSION VERSION_LESS "3.10")
        # for CMake below 3.10 we define our
        # own include_guard(GLOBAL)
        message(STATUS "calling our custom include_guard")
        # if this macro is called the first time
        # we start with an empty list
        if(NOT DEFINED included_modules)
        set(included_modules)
        endif()
        if ("${CMAKE_CURRENT_LIST_FILE}" IN_LIST included_modules)
        message(WARNING "module ${CMAKE_CURRENT_LIST_FILE} processed more than once")
        endif()
        list(APPEND included_modules ${CMAKE_CURRENT_LIST_FILE})
        else()
        # for CMake 3.10 or higher we augment
        # the built-in include_guard
        message(STATUS "calling the built-in include_guard")
        _include_guard(${ARGV})
    endif()
    endmacro()

4 主CMakeLists.txt中，我们模拟了两次包含自定义模块的情况:

.. code-block:: cmake

    include(cmake/custom.cmake)
    include(cmake/custom.cmake)

5 最后，使用以下命令进行配置:

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..

6 使用CMake 3.10及更高版本的结果如下:

.. code-block:: cmake

    -- calling the built-in include_guard
    -- custom.cmake is included and processed
    -- calling the built-in include_guard

7 使用CMake得到3.10以下的结果如下:

.. code-block:: cmake

    - calling our custom include_guard
    -- custom.cmake is included and processed
    -- calling our custom include_guard
    CMake Warning at cmake/include_guard.cmake:7 (message):
    module
    /home/user/example/cmake/custom.cmake
    processed more than once
    Call Stack (most recent call first):
    cmake/custom.cmake:1 (include_guard)
    CMakeLists.txt:12 (include)

**工作原理**

include_guard宏包含两个分支，一个用于CMake低于3.10，另一个用于CMake高于3.10:

.. code-block:: cmake

    macro(include_guard)
    if (CMAKE_VERSION VERSION_LESS "3.10")
        # ...
    else()
        # ...
    endif()
    endmacro()

如果CMake版本低于3.10，进入第一个分支，并且内置的include_guard不可用，所以我们自定义了一个:

.. code-block:: cmake

    message(STATUS "calling our custom include_guard")
    # if this macro is called the first time
    # we start with an empty list
    if(NOT DEFINED included_modules)
        set(included_modules)
    endif()
    if ("${CMAKE_CURRENT_LIST_FILE}" IN_LIST included_modules)
        message(WARNING "module ${CMAKE_CURRENT_LIST_FILE} processed more than once")
    endif()
    list(APPEND included_modules ${CMAKE_CURRENT_LIST_FILE})

如果第一次调用宏，则included_modules变量没有定义，因此我们将其设置为空列表。然后检查${CMAKE_CURRENT_LIST_FILE}
是否是included_modules列表中的元素。如果是，则会发出警告；如果没有，我们将${CMAKE_CURRENT_LIST_FILE}追加到这个列表。
CMake输出中，我们可以验证自定义模块的第二个包含确实会导致警告。

CMake 3.10及更高版本的情况有所不同；在这种情况下，存在一个内置的include_guard，我们用自己的宏接收到参数并调用它:

.. code-block:: cmake

    macro(include_guard)
    if (CMAKE_VERSION VERSION_LESS "3.10")
        # ...
    else()
        message(STATUS "calling the built-in include_guard")
        _include_guard(${ARGV})
    endif()
    endmacro()

这里，_include_guard(${ARGV})指向内置的include_guard。本例中，使用自定义消息(“调用内置的include_guard”)进行了扩展。
这种模式为我们提供了一种机制，来重新定义自己的或内置的函数和宏，这对于调试或记录日志来说非常有用。

7.6 使用废弃函数、宏和变量
-----------------------------

.. NOTE::
    
    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-06 中找到。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

“废弃”是在不断发展的项目开发过程中一种重要机制，它向开发人员发出信号，表明将来某个函数、宏或变量将被删除或替换。在一段时间内，
函数、宏或变量将继续可访问，但会发出警告，最终可能会上升为错误。

**准备工作**

我们将从以下CMake项目开始:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-06 LANGUAGES NONE)
    macro(custom_include_guard)
    if(NOT DEFINED included_modules)
        set(included_modules)
    endif()
    if ("${CMAKE_CURRENT_LIST_FILE}" IN_LIST included_modules)
        message(WARNING "module ${CMAKE_CURRENT_LIST_FILE} processed more than once")
    endif()
    list(APPEND included_modules ${CMAKE_CURRENT_LIST_FILE})
    endmacro()
    include(cmake/custom.cmake)
    message(STATUS "list of all included modules: ${included_modules}")

这段代码定义了一个自定义的”包含保护”机制，包括一个自定义模块(与前一个示例中的模块相同)，并打印所有包含模块的列表。
对于CMake 3.10或更高版本有内置的include_guard。但是，不能简单地删除custom_include_guard和${included_modules}，
而是使用一个“废弃”警告来弃用宏和变量。某个时候，可以将该警告转换为FATAL_ERROR，使代码停止配置，并迫使开发人员对代码进行修改，
切换到内置命令。

**具体实施**

“废弃”函数、宏和变量的方法如下:

1 首先，定义一个函数，我们将使用它来弃用一个变量:

.. code-block:: cmake

    function(deprecate_variable _variable _access)
    if(_access STREQUAL "READ_ACCESS")
        message(DEPRECATION "variable ${_variable} is deprecated")
    endif()
    endfunction()

2 然后，如果CMake的版本大于3.9，我们重新定义custom_include_guard并将variable_watch附加到included_modules中:

.. code-block:: cmake

    if (CMAKE_VERSION VERSION_GREATER "3.9")
    # deprecate custom_include_guard
    macro(custom_include_guard)
        message(DEPRECATION "custom_include_guard is deprecated - use built-in include_guard instead")
        _custom_include_guard(${ARGV})
    endmacro()
    # deprecate variable included_modules
    variable_watch(included_modules deprecate_variable)
    endif()

3 CMake3.10以下版本的项目会产生以下结果:

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..
    -- custom.cmake is included and processed
    -- list of all included modules: /home/user/example/cmake/custom.cmake

4 CMake 3.10及以上将产生预期的“废弃”警告:

.. code-block:: cmake

    CMake Deprecation Warning at CMakeLists.txt:26 (message):
    custom_include_guard is deprecated - use built-in include_guard instead
    Call Stack (most recent call first):
    cmake/custom.cmake:1 (custom_include_guard)
    CMakeLists.txt:34 (include)
    -- custom.cmake is included and processed
    CMake Deprecation Warning at CMakeLists.txt:19 (message):
    variable included_modules is deprecated
    Call Stack (most recent call first):
    CMakeLists.txt:9999 (deprecate_variable)
    CMakeLists.txt:36 (message)
    -- list of all included modules: /home/user/example/cmake/custom.cmake

**工作原理**

弃用函数或宏相当于重新定义它，如前面的示例所示，并使用DEPRECATION打印消息:

.. code-block:: cmake

    macro(somemacro)
    message(DEPRECATION "somemacro is deprecated")
    _somemacro(${ARGV})
    endmacro()

可以通过定义以下变量来实现对变量的弃用:

.. code-block:: cmake

    function(deprecate_variable _variable _access)
    if(_access STREQUAL "READ_ACCESS")
        message(DEPRECATION "variable ${_variable} is deprecated")
    endif()
    endfunction()

然后，这个函数被添加到将要“废弃”的变量上:

.. code-block:: cmake

    variable_watch(somevariable deprecate_variable)

如果在本例中${included_modules}是读取 (READ_ACCESS)，那么deprecate_variable函数将发出带有DEPRECATION的消息。

7.7 add_subdirectory的限定范围
-------------------------------------

.. NOTE::
    
    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-07 中找到，其中有一个C++示例。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

本章剩下的示例中，我们将讨论构建项目的策略，并限制变量的范围和副作用，目的是降低代码的复杂性和简化项目的维护。
这个示例中，我们将把一个项目分割成几个范围有限的CMakeLists.txt文件，这些文件将使用add_subdirectory命令进行处理。

**准备工作**

由于我们希望展示和讨论如何构造一个复杂的项目，所以需要一个比“hello world”项目更复杂的例子:

https://en.wikipedia.org/wiki/Cellular_automaton#Elementary_cellular_automata
http://mathworld.wolfram.com/ElementaryCellularAutomaton.html

我们的代码将能够计算任何256个基本细胞自动机，例如：规则90 (Wolfram代码):


我们示例代码项目的结构如下:

.. code-block:: cmake

    .
    ├── CMakeLists.txt
    ├── external
    │    ├── CMakeLists.txt
    │    ├── conversion.cpp
    │    ├── conversion.hpp
    │    └── README.md
    ├── src
    │    ├── CMakeLists.txt
    │    ├── evolution
    │    │    ├── CMakeLists.txt
    │    │    ├── evolution.cpp
    │    │    └── evolution.hpp
    │    ├── initial
    │    │    ├── CMakeLists.txt
    │    │    ├── initial.cpp
    │    │    └── initial.hpp
    │    ├── io
    │    │    ├── CMakeLists.txt
    │    │    ├── io.cpp
    │    │    └── io.hpp
    │    ├── main.cpp
    │    └── parser
    │        ├── CMakeLists.txt
    │        ├── parser.cpp
    │        └── parser.hpp
    └── tests
        ├── catch.hpp
        ├── CMakeLists.txt
        └── test.cpp

我们将代码分成许多库来模拟真实的大中型项目，可以将源代码组织到库中，然后将库链接到可执行文件中。

主要功能在src/main.cpp中:

.. code-block:: cmake

    #include "conversion.hpp"
    #include "evolution.hpp"
    #include "initial.hpp"
    #include "io.hpp"
    #include "parser.hpp"
    #include <iostream>
    int main(int argc, char *argv[]) {
    // parse arguments
    int length, num_steps, rule_decimal;
    std::tie(length, num_steps, rule_decimal) = parse_arguments(argc, argv);
    // print information about parameters
    std::cout << "length: " << length << std::endl;
    std::cout << "number of steps: " << num_steps << std::endl;
    std::cout << "rule: " << rule_decimal << std::endl;
    // obtain binary representation for the rule
    std::string rule_binary = binary_representation(rule_decimal);
    // create initial distribution
    std::vector<int> row = initial_distribution(length);
    // print initial configuration
    print_row(row);
    // the system evolves, print each step
    for (int step = 0; step < num_steps; step++) {
        row = evolve(row, rule_binary);
        print_row(row);
    }
    }

external/conversion.cpp文件包含要从十进制转换为二进制的代码。

我们在这里模拟这段代码是由src外部的“外部”库提供的:

.. code-block:: cmake

    #include "conversion.hpp"
    #include <bitset>
    #include <string>
    std::string binary_representation(const int decimal) {
        return std::bitset<8>(decimal).to_string();
    }

src/evolution/evolution.cpp文件为一个时限传播系统:

.. code-block:: cmake

    #include "evolution.hpp"
    #include <string>
    #include <vector>
    std::vector<int> evolve(const std::vector<int> row, const std::string rule_binary) {
    std::vector<int> result;
    for (auto i = 0; i < row.size(); ++i) {
        auto left = (i == 0 ? row.size() : i) - 1;
        auto center = i;
        auto right = (i + 1) % row.size();
        auto ancestors = 4 * row[left] + 2 * row[center] + 1 * row[right];
        ancestors = 7 - ancestors;
        auto new_state = std::stoi(rule_binary.substr(ancestors, 1));
        result.push_back(new_state);
    }
    return result;
    }

src/initial/initial.cpp文件，对出进行初始化:

.. code-block:: cmake

    #include "initial.hpp"
    #include <vector>
    std::vector<int> initial_distribution(const int length) {
    // we start with a vector which is zeroed out
    std::vector<int> result(length, 0);
    // more or less in the middle we place a living cell
    result[length / 2] = 1;
    return result;
    }

src/io/io.cpp文件包含一个函数输出打印行:

.. code-block:: cmake

    #include "io.hpp"
    #include <algorithm>
    #include <iostream>
    #include <vector>
    void print_row(const std::vector<int> row) {
    std::for_each(row.begin(), row.end(), [](int const &value) {
        std::cout << (value == 1 ? '*' : ' ');
    });
    std::cout << std::endl;
    }

src/parser/parser.cpp文件解析命令行输入:

.. code-block:: cmake

    #include "parser.hpp"
    #include <cassert>
    #include <string>
    #include <tuple>
    std::tuple<int, int, int> parse_arguments(int argc, char *argv[]) {
    assert(argc == 4 && "program called with wrong number of arguments");
    auto length = std::stoi(argv[1]);
    auto num_steps = std::stoi(argv[2]);
    auto rule_decimal = std::stoi(argv[3]);
    return std::make_tuple(length, num_steps, rule_decimal);
    }

最后，tests/test.cpp包含两个使用Catch2库的单元测试:

.. code-block:: cmake
    
    #include "evolution.hpp"
    // this tells catch to provide a main()
    // only do this in one cpp file
    #define CATCH_CONFIG_MAIN
    #include "catch.hpp"
    #include <string>
    #include <vector>
    TEST_CASE("Apply rule 90", "[rule-90]") {
    std::vector<int> row = {0, 1, 0, 1, 0, 1, 0, 1, 0};
    std::string rule = "01011010";
    std::vector<int> expected_result = {1, 0, 0, 0, 0, 0, 0, 0, 1};
    REQUIRE(evolve(row, rule) == expected_result);
    }
    TEST_CASE("Apply rule 222", "[rule-222]") {
    std::vector<int> row = {0, 0, 0, 0, 1, 0, 0, 0, 0};
    std::string rule = "11011110";
    std::vector<int> expected_result = {0, 0, 0, 1, 1, 1, 0, 0, 0};
    REQUIRE(evolve(row, rule) == expected_result);
    }

相应的头文件包含函数声明。有人可能会说，对于这个小代码示例，项目包含了太多子目录。请注意，这只是一个项目的简化示例，
通常包含每个库的许多源文件，理想情况下，这些文件被放在到单独的目录中。

**具体实施**

让我们来详细解释一下CMake所需的功能:

1 CMakeLists.txt顶部非常类似于第1节，代码重用与函数和宏:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-07 LANGUAGES CXX)
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
    # defines targets and sources
    add_subdirectory(src)
    # contains an "external" library we will link to
    add_subdirectory(external)
    # enable testing and define tests
    enable_testing()
    add_subdirectory(tests)

2 目标和源在src/CMakeLists.txt中定义(转换目标除外):

.. code-block:: cmake

    add_executable(automata main.cpp)
    add_subdirectory(evolution)
    add_subdirectory(initial)
    add_subdirectory(io)
    add_subdirectory(parser)
    target_link_libraries(automata
    PRIVATE
        conversion
        evolution
        initial
        io
        parser
    )

3 转换库在external/CMakeLists.txt中定义:

.. code-block:: cmake

    add_library(conversion "")
    target_sources(conversion
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/conversion.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/conversion.hpp
    )
    target_include_directories(conversion
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

4 src/CMakeLists.txt文件添加了更多的子目录，这些子目录又包含CMakeLists.txt文件。src/evolution/CMakeLists.txt包含以下内容:

.. code-block:: cmake

    add_library(evolution "")
    target_sources(evolution
    PRIVATE
        evolution.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/evolution.hpp
    )
    target_include_directories(evolution
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

5 单元测试在tests/CMakeLists.txt中注册:

.. code-block:: cmake

    add_executable(cpp_test test.cpp)
    target_link_libraries(cpp_test evolution)
    add_test(
    NAME
        test_evolution
    COMMAND
        $<TARGET_FILE:cpp_test>
    )

6 配置和构建项目产生以下输出:

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..
    $ cmake --build .
    Scanning dependencies of target conversion
    [ 7%] Building CXX object external/CMakeFiles/conversion.dir/conversion.cpp.o
    [ 14%] Linking CXX static library ../lib64/libconversion.a
    [ 14%] Built target conversion
    Scanning dependencies of target evolution
    [ 21%] Building CXX object src/evolution/CMakeFiles/evolution.dir/evolution.cpp.o
    [ 28%] Linking CXX static library ../../lib64/libevolution.a
    [ 28%] Built target evolution
    Scanning dependencies of target initial
    [ 35%] Building CXX object src/initial/CMakeFiles/initial.dir/initial.cpp.o
    [ 42%] Linking CXX static library ../../lib64/libinitial.a
    [ 42%] Built target initial
    Scanning dependencies of target io
    [ 50%] Building CXX object src/io/CMakeFiles/io.dir/io.cpp.o
    [ 57%] Linking CXX static library ../../lib64/libio.a
    [ 57%] Built target io
    Scanning dependencies of target parser
    [ 64%] Building CXX object src/parser/CMakeFiles/parser.dir/parser.cpp.o
    [ 71%] Linking CXX static library ../../lib64/libparser.a
    [ 71%] Built target parser
    Scanning dependencies of target automata
    [ 78%] Building CXX object src/CMakeFiles/automata.dir/main.cpp.o
    [ 85%] Linking CXX executable ../bin/automata
    [ 85%] Built target automata
    Scanning dependencies of target cpp_test
    [ 92%] Building CXX object tests/CMakeFiles/cpp_test.dir/test.cpp.o
    [100%] Linking CXX executable ../bin/cpp_test
    [100%] Built target cpp_test

7 最后，运行单元测试:

.. code-block:: cmake

    $ ctest
    Running tests...
    Start 1: test_evolution
    1/1 Test #1: test_evolution ................... Passed 0.00 sec
    100% tests passed, 0 tests failed out of 1

**工作原理**

我们可以将所有代码放到一个源文件中。不过，每次编辑都需要重新编译。将源文件分割成更小、更易于管理的单元是有意义的。
可以将所有源代码都编译成一个库或可执行文件。实际上，项目更喜欢将源代码编译分成更小的、定义良好的库。这样做既是为了本地化和简化依赖项，
也是为了简化代码维护。这意味着如在这里所做的那样，由许多库构建一个项目是一种常见的情况。

为了讨论CMake结构，我们可以从定义每个库的单个CMakeLists.txt文件开始，自底向上进行，例如src/evolution/CMakeLists.txt:

.. code-block:: cmake

    add_library(evolution "")
    target_sources(evolution
    PRIVATE
        evolution.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/evolution.hpp
    )
    target_include_directories(evolution
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

这些单独的CMakeLists.txt文件定义了库。本例中，我们首先使用add_library定义库名，然后定义它的源和包含目录，以及它们的目标可见性：
实现文件(evolution.cpp:PRIVATE)，而接口头文件evolution.hpp定义为PUBLIC，因为我们将在main.cpp和test.cpp中访问它。
定义尽可能接近代码目标的好处是，对于该库的修改，只需要变更该目录中的文件即可；换句话说，也就是库依赖项被封装。

向上移动一层，库在src/CMakeLists.txt中封装:

.. code-block:: cmake

    add_executable(automata main.cpp)
    add_subdirectory(evolution)
    add_subdirectory(initial)
    add_subdirectory(io)
    add_subdirectory(parser)
    target_link_libraries(automata
    PRIVATE
        conversion
        evolution
        initial
        io
        parser
    )

文件在主CMakeLists.txt中被引用。这意味着使用CMakeLists.txt文件，构建我们的项目。这种方法对于许多项目来说是可用的，
并且它可以扩展到更大型的项目，而不需要在目录间的全局变量中包含源文件列表。add_subdirectory方法的另一个好处是它隔离了作用范围，
因为子目录中定义的变量在父范围中不能访问。


7.8 使用target_sources避免全局变量
-----------------------------------------

.. NOTE::
    
    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-08 中找到，其中有一个C++示例。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

本示例中，我们将讨论前一个示例的另一种方法，并不使用add_subdirectory的情况下，使用module include组装不同的CMakeLists.txt文件。
这种方法的灵感来自https://crascit.com/2016/01/31/enhance-sours-file-handling-with-target_sources/ ，
其允许我们使用target_link_libraries链接到当前目录之外定义的目标。

**准备工作**

将使用与前一个示例相同的源代码。惟一的更改将出现在CMakeLists.txt文件中，我们将在下面的部分中讨论这些更改。

**具体实施**

1 主CMakeLists.txt包含以下内容:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-08 LANGUAGES CXX)
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
    # defines targets and sources
    include(src/CMakeLists.txt)
    include(external/CMakeLists.txt)
    enable_testing()
    add_subdirectory(tests)

2 与前一个示例相比，external/CMakeLists.txt文件没有变化。

3 src/CMakeLists.txt文件定义了两个库(automaton和evolution):

.. code-block:: cmake

    add_library(automaton "")
    add_library(evolution "")
    include(${CMAKE_CURRENT_LIST_DIR}/evolution/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/initial/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/io/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/parser/CMakeLists.txt)
    add_executable(automata "")
    target_sources(automata
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    )
    target_link_libraries(automata
    PRIVATE
        automaton
        conversion
    )
    src/evolution/CMakeLists.txt文件包含以下内容:

    target_sources(automaton
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/evolution.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/evolution.hpp
    )
    target_include_directories(automaton
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )
    target_sources(evolution
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/evolution.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/evolution.hpp
    )
    target_include_directories(evolution
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

4 其余CMakeLists.txt文件和src/initial/CMakeLists.txt相同:

.. code-block:: cmake

    target_sources(automaton
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/initial.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/initial.hpp
    )
    target_include_directories(automaton
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

5 配置、构建和测试的结果与前面的方法相同:

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..
    $ cmake --build build
    $ ctest
    Running tests...
    Start 1: test_evolution
    1/1 Test #1: test_evolution ................... Passed 0.00 sec
    100% tests passed, 0 tests failed out of 1

**工作原理**

与之前的示例不同，我们定义了三个库:

.. code-block:: cmake

    conversion(在external定义)
    automaton(包含除转换之外的所有源)
    evolution(在src/evolution中定义，并通过cpp_test链接)

本例中，通过使用include()引用CMakeLists.txt文件，我们在父范围内，仍然能保持所有目标可用:

.. code-block:: cmake

    include(src/CMakeLists.txt)
    include(external/CMakeLists.txt)

我们可以构建一个包含树，记住当进入子目录(src/CMakeLists.txt)时，我们需要使用相对于父范围的路径:

.. code-block:: cmake

    include(${CMAKE_CURRENT_LIST_DIR}/evolution/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/initial/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/io/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/parser/CMakeLists.txt)

这样，我们就可以定义并链接到通过include()语句访问文件树中任何位置的目标。但是，我们应该选择在对维护人员和代码贡献者容易看到的地方，
去定义它们。

7.9 组织Fortran项目
--------------------------

.. NOTE:：

    此示例代码可以在 https://github.com/dev-cafe/cmake-cookbook/tree/v1.0/chapter-7/recipe-09 中找到，其中有一个Fortran示例。
    该示例在CMake 3.5版(或更高版本)中是有效的，并且已经在GNU/Linux、macOS和Windows上进行过测试。

我们来讨论如何构造和组织Fortran项目，原因有二:

现在，仍然有很多Fortran项目，特别是在数字软件中(有关通用Fortran软件项目的更全面列表，
请参见http://fortranwiki.org/fortran/show/Libraries )。
对于不使用CMake的项目，Fortran 90(以及更高版本)可能更难构建，因为Fortran模块强制执行编译顺序。
换句话说，对于手工编写的Makefile，通常需要为Fortran模块文件编写依赖扫描程序。
正如我们在本示例中所示，现代CMake允许我们以非常紧凑和模块化的方式配置和构建项目。作为一个例子，我们将使用前两个示例中的基本元胞自动机，
现在将其移植到Fortran。

**准备工作**

文件树结构与前两个示例非常相似。我们用Fortran源代码替换了C++，现在就没有头文件了:

.. code-block:: cmake

    .
    ├── CMakeLists.txt
    ├── external
    │    ├── CMakeLists.txt
    │    ├── conversion.f90
    │    └── README.md
    ├── src
    │    ├── CMakeLists.txt
    │    ├── evolution
    │    │    ├── ancestors.f90
    │    │    ├── CMakeLists.txt
    │    │    ├── empty.f90
    │    │    └── evolution.f90
    │    ├── initial
    │    │    ├── CMakeLists.txt
    │    │    └── initial.f90
    │    ├── io
    │    │    ├── CMakeLists.txt
    │    │    └── io.f90
    │    ├── main.f90
    │    └── parser
    │        ├── CMakeLists.txt
    │        └── parser.f90
    └── tests
        ├── CMakeLists.txt
        └── test.f90

主程序在src/main.f90中:

.. code-block:: cmake

    program example
    use parser, only: get_arg_as_int
    use conversion, only: binary_representation
    use initial, only: initial_distribution
    use io, only: print_row
    use evolution, only: evolve
    implicit none
    integer :: num_steps
    integer :: length
    integer :: rule_decimal
    integer :: rule_binary(8)
    integer, allocatable :: row(:)
    integer :: step
    ! parse arguments
    num_steps = get_arg_as_int(1)
    length = get_arg_as_int(2)
    rule_decimal = get_arg_as_int(3)
    ! print information about parameters
    print *, "number of steps: ", num_steps
    print *, "length: ", length
    print *, "rule: ", rule_decimal
    ! obtain binary representation for the rule
    rule_binary = binary_representation(rule_decimal)
    ! create initial distribution
    allocate(row(length))
    call initial_distribution(row)
    ! print initial configuration
    call print_row(row)
    ! the system evolves, print each step
    do step = 1, num_steps
        call evolve(row, rule_binary)
        call print_row(row)
    end do
    deallocate(row)
    end program

与前面的示例一样，我们已经将conversion模块放入external/conversion.f90中：

.. code-block:: cmake

    module conversion
    implicit none
    public binary_representation
    private
    contains
    pure function binary_representation(n_decimal)
        integer, intent(in) :: n_decimal
        integer :: binary_representation(8)
        integer :: pos
        integer :: n
        binary_representation = 0
        pos = 8
        n = n_decimal
        do while (n > 0)
        binary_representation(pos) = mod(n, 2)
        n = (n - binary_representation(pos))/2
        pos = pos - 1
        end do
    end function
    end module

evolution库分成三个文件，大部分在src/evolution/evolution.f90中:

.. code-block:: cmake

    module evolution
    implicit none
    public evolve
    private
    contains
    subroutine not_visible()
        ! no-op call to demonstrate private/public visibility
        call empty_subroutine_no_interface()
    end subroutine
    pure subroutine evolve(row, rule_binary)
        use ancestors, only: compute_ancestors
        integer, intent(inout) :: row(:)
        integer, intent(in) :: rule_binary(8)
        integer :: i
        integer :: left, center, right
        integer :: ancestry
        integer, allocatable :: new_row(:)
        allocate(new_row(size(row)))
        do i = 1, size(row)
        left = i - 1
        center = i
        right = i + 1
        if (left < 1) left = left + size(row)
        if (right > size(row)) right = right - size(row)
        ancestry = compute_ancestors(row, left, center, right)
        new_row(i) = rule_binary(ancestry)
        end do
        row = new_row
        deallocate(new_row)
    end subroutine
    end module

祖先计算是在src/evolution/ancestors.f90：

.. code-block:: cmake

    module ancestors
    implicit none
    public compute_ancestors
    private
    contains
    pure integer function compute_ancestors(row, left, center, right) result(i)
        integer, intent(in) :: row(:)
        integer, intent(in) :: left, center, right
        i = 4*row(left) + 2*row(center) + 1*row(right)
        i = 8 - i
    end function
    end module

还有一个“空”模块在src/evolution/empty.f90中：

.. code-block:: cmake

    module empty
    implicit none
    public empty_subroutine
    private
    contains
    subroutine empty_subroutine()
    end subroutine
    end module
    subroutine 
    empty_subroutine_no_interface()
    use empty, only: empty_subroutine
    call empty_subroutine()
    end subroutine

启动条件的代码位于src/initial/initial.f90：

.. code-block:: cmake

    module initial
    implicit none
    public initial_distribution
    private
    contains
        pure subroutine initial_distribution(row)
        integer, intent(out) :: row(:)
        row = 0
        row(size(row)/2) = 1
        end subroutine
    end module

src/io/io.f90包含一个打印输出：

.. code-block:: cmake

    module io
    implicit none
    public print_row
    private
    contains
    subroutine print_row(row)
        integer, intent(in) :: row(:)
        character(size(row)) :: line
        integer :: i
        do i = 1, size(row)
        if (row(i) == 1) then
            line(i:i) = '*'
        else
            line(i:i) = ' '
        end if
        end do
        print *, line
    end subroutine
    end module

src/parser/parser.f90用于解析命令行参数：

.. code-block:: cmake

    module parser
    implicit none
    public get_arg_as_int
    private
    contains
    integer function get_arg_as_int(n) result(i)
        integer, intent(in) :: n
        character(len=32) :: arg
        call get_command_argument(n, arg)
        read(arg , *) i
    end function
    end module

最后，使用tests/test.f90对上面的实现进行测试：

.. code-block:: cmake

    program test
    use evolution, only: evolve
    implicit none
    integer :: row(9)
    integer :: expected_result(9)
    integer :: rule_binary(8)
    integer :: i
    ! test rule 90
    row = (/0, 1, 0, 1, 0, 1, 0, 1, 0/)
    rule_binary = (/0, 1, 0, 1, 1, 0, 1, 0/)
    call evolve(row, rule_binary)
    expected_result = (/1, 0, 0, 0, 0, 0, 0, 0, 1/)
    do i = 1, 9
        if (row(i) /= expected_result(i)) then
            print *, 'ERROR: test for rule 90 failed'
            call exit(1)
        end if
    end do
    ! test rule 222
    row = (/0, 0, 0, 0, 1, 0, 0, 0, 0/)
    rule_binary = (/1, 1, 0, 1, 1, 1, 1, 0/)
    call evolve(row, rule_binary)
    expected_result = (/0, 0, 0, 1, 1, 1, 0, 0, 0/)
    do i = 1, 9
        if (row(i) /= expected_result(i)) then
            print *, 'ERROR: test for rule 222 failed'
            call exit(1)
        end if
    end do
    end program

**具体实施**

1 主CMakeLists.txt类似于第7节，我们只是将CXX换成Fortran，去掉C++11的要求:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(recipe-09 LANGUAGES Fortran)
    include(GNUInstallDirs)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
    # defines targets and sources
    add_subdirectory(src)
    # contains an "external" library we will link to
    add_subdirectory(external)
    # enable testing and define tests
    enable_testing()
    add_subdirectory(tests)

2 目标和源在src/CMakeLists.txt中定义(conversion目标除外):

.. code-block:: cmake

    add_executable(automata main.f90)
    add_subdirectory(evolution)
    add_subdirectory(initial)
    add_subdirectory(io)
    add_subdirectory(parser)
    target_link_libraries(automata
    PRIVATE
        conversion
        evolution
        initial
        io
        parser
    )

3 conversion库在external/CMakeLists.txt中定义:

.. code-block:: cmake

    add_library(conversion "")
    target_sources(conversion
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/conversion.f90
    )

4 src/CMakeLists.txt文件添加了更多的子目录，这些子目录又包含CMakeLists.txt文件。它们在结构上都是相似的，例如：src/initial/CMakeLists.txt包含以下内容:

.. code-block:: cmake

    add_library(initial "")
    target_sources(initial
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/initial.f90
    )

5 有个例外的是src/evolution/CMakeLists.txt中的evolution库，我们将其分为三个源文件:

.. code-block:: cmake

    add_library(evolution "")
    target_sources(evolution
    PRIVATE
        empty.f90
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/ancestors.f90
        ${CMAKE_CURRENT_LIST_DIR}/evolution.f90
    )

6 单元测试在tests/CMakeLists.txt中注册:

.. code-block:: cmake

    add_executable(fortran_test test.f90)
    target_link_libraries(fortran_test evolution)
    add_test(
    NAME
        test_evolution
    COMMAND
        $<TARGET_FILE:fortran_test>
    )

7 配置和构建项目，将产生以下输出:

.. code-block:: cmake

    $ mkdir -p build
    $ cd build
    $ cmake ..
    $ cmake --build .
    Scanning dependencies of target conversion
    [ 4%] Building Fortran object external/CMakeFiles/conversion.dir/conversion.f90.o
    [ 8%] Linking Fortran static library ../lib64/libconversion.a
    [ 8%] Built target conversion
    Scanning dependencies of target evolution
    [ 12%] Building Fortran object src/evolution/CMakeFiles/evolution.dir/ancestors.f90.o
    [ 16%] Building Fortran object src/evolution/CMakeFiles/evolution.dir/empty.f90.o
    [ 20%] Building Fortran object src/evolution/CMakeFiles/evolution.dir/evolution.f90.o
    [ 25%] Linking Fortran static library ../../lib64/libevolution.a
    [ 25%] Built target evolution
    Scanning dependencies of target initial
    [ 29%] Building Fortran object src/initial/CMakeFiles/initial.dir/initial.f90.o
    [ 33%] Linking Fortran static library ../../lib64/libinitial.a
    [ 33%] Built target initial
    Scanning dependencies of target io
    [ 37%] Building Fortran object src/io/CMakeFiles/io.dir/io.f90.o
    [ 41%] Linking Fortran static library ../../lib64/libio.a
    [ 41%] Built target io
    Scanning dependencies of target parser
    [ 45%] Building Fortran object src/parser/CMakeFiles/parser.dir/parser.f90.o
    [ 50%] Linking Fortran static library ../../lib64/libparser.a
    [ 50%] Built target parser
    Scanning dependencies of target example
    [ 54%] Building Fortran object src/CMakeFiles/example.dir/__/external/conversion.f90.o
    [ 58%] Building Fortran object src/CMakeFiles/example.dir/evolution/ancestors.f90.o
    [ 62%] Building Fortran object src/CMakeFiles/example.dir/evolution/evolution.f90.o
    [ 66%] Building Fortran object src/CMakeFiles/example.dir/initial/initial.f90.o
    [ 70%] Building Fortran object src/CMakeFiles/example.dir/io/io.f90.o
    [ 75%] Building Fortran object src/CMakeFiles/example.dir/parser/parser.f90.o
    [ 79%] Building Fortran object src/CMakeFiles/example.dir/main.f90.o
    [ 83%] Linking Fortran executable ../bin/example
    [ 83%] Built target example
    Scanning dependencies of target fortran_test
    [ 87%] Building Fortran object tests/CMakeFiles/fortran_test.dir/__/src/evolution/ancestors.f90.o
    [ 91%] Building Fortran object tests/CMakeFiles/fortran_test.dir/__/src/evolution/evolution.f90.o
    [ 95%] Building Fortran object tests/CMakeFiles/fortran_test.dir/test.f90.o
    [100%] Linking Fortran executable

8 最后，运行单元测试：

.. code-block:: cmake

    $ ctest
    Running tests...
    Start 1: test_evolution
    1/1 Test #1: test_evolution ................... Passed 0.00 sec
    100% tests passed, 0 tests failed out of 1

**工作原理**

第7节中使用add_subdirectory限制范围，将从下往上讨论CMake结构，从定义每个库的单个CMakeLists.txt文件开始，
比如src/evolution/CMakeLists.txt:

.. code-block:: cmake

    add_library(evolution "")
    target_sources(evolution
    PRIVATE
        empty.f90
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/ancestors.f90
        ${CMAKE_CURRENT_LIST_DIR}/evolution.f90
    )

这些独立的CMakeLists.txt文件定义了源文件的库，遵循与前两个示例相同的方式：开发或维护人员可以对其中文件分而治之。

首先用add_library定义库名，然后定义它的源和包含目录，以及它们的目标可见性。这种情况下，因为它们的模块接口是在库之外访问，
所以ancestors.f90和evolution.f90都是PUBLIC，而模块接口empty.f90不能在文件之外访问，因此将其标记为PRIVATE。

向上移动一层，库在src/CMakeLists.txt中封装：

.. code-block:: cmake

    add_executable(automata main.f90)
    add_subdirectory(evolution)
    add_subdirectory(initial)
    add_subdirectory(io)
    add_subdirectory(parser)
    target_link_libraries(automata
    PRIVATE
        conversion
        evolution
        initial
        io
        parser
    )

这个文件在主CMakeLists.txt中被引用。这意味着我们使用CMakeLists.txt文件(使用add_subdirectory添加)构建项目。
正如第7节中讨论的，使用add_subdirectory限制范围，这种方法可以扩展到更大型的项目，而不需要在多个目录之间的全局变量中携带源文件列表，
还可以隔离范围和名称空间。

将这个Fortran示例与C++版本(第7节)进行比较，我们可以注意到，在Fortran的情况下，相对的CMake工作量比较小；
我们不需要使用target_include_directory，因为没有头文件，接口是通过生成的Fortran模块文件进行通信。另外，
我们既不需要担心target_sources中列出的源文件的顺序，也不需要在库之间强制执行任何显式依赖关系。CMake能够从源文件依赖项推断Fortran
模块依赖项。使用target_sources与PRIVATE和PUBLIC资源结合使用，以紧凑和健壮的方式表示接口。

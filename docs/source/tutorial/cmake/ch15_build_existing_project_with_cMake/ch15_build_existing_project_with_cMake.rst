.. highlight:: c++

.. default-domain:: cpp

============================
第15章 使用CMake构建已有项目
============================


15.1 如何开始迁移项目
-----------------------------

我们将首先说明，在哪里可以找到我们的示例，然后对移植，进行逐步的讨论。

**复制要移植的示例**

我们将从Vim源代码库的v8.1.0290发行标记开始(https://github.com/vim/vim) ，我们的工作基于Git提交哈希值b476cb7进行。
通过克隆Vim的源代码库并检出特定版本的代码，可以复制以下步骤:

.. code-block:: bash

  $ git clone --single-branch -b v8.1.0290 https://github.com/vim/vim.git

或者，我们的解决方案可以在cmake-support分支上找到，网址是 https://github.com/dev-cafe/vim ，并使用以下方法克隆下来:

.. code-block:: bash

  $ git clone --single-branch -b cmake-support https://github.com/dev-cafe/vim

在本例中，我们将使用CMake模拟./configure --enable-gui=no的配置方式。

为了与后面的解决方案进行比较，建议读者也可以研究以下Neovim项目(https://github.com/neovim/neovim )，这是传统Vi编辑器的一个分支，提供了一个CMake构建系统。

**创建一个主CMakeLists.txt**

首先，我们在源代码存储库的根目录中创建主CMakeLists.txt，在这里我们设置了最低CMake版本、项目名称和支持的语言，在本例中是C：

.. code-block:: bash

  cmake_minimum_required(VERSION
  3.5 FATAL_ERROR)
  project(vim LANGUAGES C)

添加任何目标或源之前，可以设置默认的构建类型。本例中，我们默认为Release配置，这将打开某些编译器优化选项:

.. code-block:: bash

  if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()

我们也使用可移植的安装目录变量：

.. code-block:: bash

  include(GNUInstallDirs)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

作为一个完整性检查，我们可以尝试配置和构建项目，但到目前为止还没有目标，所以构建步骤的输出是空的:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .

我们一会儿就要开始添加目标了。

**如何让常规和CMake配置共存**

CMake的一个特性是在源代码之外构建，构建目录可以是任何目录，而不必是项目目录的子目录。这意味着，我们可以将一个项目移植到CMake，
而不影响以前/现在的配置和构建机制。对于一个重要项目的迁移，CMake文件可以与其他构建框架共存，从而允许一个渐进的迁移，包括选项、特性和可移植性，
并允许开发社区人员适应新的框架。为了允许传统配置和CMake配置共存一段时间，一个典型的策略是收集CMakeLists.txt文件中的所有CMake代码，
以及CMake子目录下的所有辅助CMake源文件的示例中，我们不会引入CMake子目录，而是保持辅助文件要求他们接近目标和来源，
但会顾及使用的传统Autotools构建修改的所有文件，但有一个例外：我们将一些修改自动生成文件构建目录下，而不是在源代码树中。

.. code-block:: bash

  $ ./configure --enable-gui=no
  ... lot of output ...
  $ make > build.log

我们的示例中(这里没有显示build.log的内容)，我们能够验证编译了哪些源文件以及使用了哪些编译标志
(-I. -Iproto -DHAVE_CONFIG_H -g -O2 -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=1)。日志文件中，我们可以做如下推断:

* 所有对象文件都链接到二进制文件中
* 不生成库
* 可执行目标与下列库进行连接:-lSM -lICE -lXpm -lXt -lX11 -lXdmcp -lSM -lICE -lm -ltinfo -lelf -lnsl -lacl -lattr -lgpm -ldl

通过在使用message对工程进行调试时，选择添加选项、目标、源和依赖项，我们将逐步实现一个可工作的构建。

**获取传统构建的记录**

向配置添加任何目标之前，通常有必要看看传统构建的行为，并将配置和构建步骤的输出保存到日志文件中。对于我们的Vim示例，可以使用以下方法实现:

.. code-block:: bash

  $ ./configure --enable-gui=no
  ... lot of output ...
  $ make > build.log

示例中(这里没有显示build.log的完整内容)，我们能够验证编译了哪些源文件以及使用了哪些编译标志
(-I.-Iproto -DHAVE_CONFIG_H -g -O2 -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=1)。从日志文件中，推断如下:

* 所有对象文件都链接到一个二进制文件中
* 没有生成库
* 可执行目标链接到以下库:-lSM -lXpm -lXt -lX11 -lXdmcp -lSM -lSM - linfo -lelf -lnsl -lacl -lattr -lgpm -ldl

**调试迁移项目**

当目标和命令逐渐移动到CMake端时，使用message命令打印变量的值就非常有用了:

.. code-block:: bash

  message(STATUS "for debugging printing the value of ${some_variable}")

在使用消息进行调试时，添加选项、目标、源和依赖项，我们将逐步实现一个可工作的构建。

**实现选项**

找出传统配置为用户提供的选项(例如，通过./configure --help)。Vim项目提供了一个非常长的选项和标志列表，为了使本章的讨论保持简单，我们只在CMake端实现四个选项:

.. code-block:: bash

  --disable-netbeans Disable NetBeans integration support.
  --disable-channel Disable process communication support.
  --enable-terminal Enable terminal emulation support.
  --with-features=TYPE tiny, small, normal, big or huge (default: huge)

我们还将忽略任何GUI支持和模拟--enable-gui=no，因为它将使示例复杂化。

我们将在CMakeLists.txt中添加以下选项(有默认值)：

.. code-block:: bash

  option(ENABLE_NETBEANS "Enable netbeans" ON)
  option(ENABLE_CHANNEL "Enable channel" ON)
  option(ENABLE_TERMINAL "Enable terminal" ON)

我们可以用cmake -D FEATURES=value定义的变量FEATURES来模拟--with-features标志。如果不进行设置，它默认值为”huge”:

.. code-block:: bash

  if(NOT FEATURES)
      set(FEATURES "huge" CACHE STRING
  "FEATURES chosen by the user at CMake configure time")
  endif()

我们为使用者提供了一个值FEATURES:

.. code-block:: bash

  list(APPEND _available_features "tiny" "small" "normal" "big" "huge")
  if(NOT FEATURES IN_LIST _available_features)
      message(FATAL_ERROR "Unknown features: \"${FEATURES}\". Allowed values are: ${_available_features}.")
  endif()

  set_property(CACHE FEATURES PROPERTY STRINGS ${_available_features})

最后一行set_property(CACHE FEATURES PROPERTY STRINGS ${_available_features})，当使用cmake-gui配置项目，
则有有不错的效果，用户可根据选择字段清单，选择已经定义了的FEATURES
(参见https://blog.kitware.com/constraining-values-with-comboboxes-in-cmake-cmake-gui/ )。

选项可以放在主CMakeLists.txt中，也可以在查询ENABLE_NETBEANS、ENABLE_CHANNEL、ENABLE_TERMINAL和FEATURES的定义附近。前一种策略的优点是，选项列在一个地方，不需要遍历CMakeLists.txt文件来查找选项的定义。因为我们还没有定义任何目标，所以可以先将选项保存在一个文件中，但是稍后会将选项移到离目标更近的地方，通过本地化作用域，得到可重用的CMake构建块。

**从可执行的目标开始，进行本地化**

让我们添加一些源码。在Vim示例中，源文件位于src下，为了保持主CMakeLists.txt的可读性和可维持性，我们将创建一个新文件src/CMakeLists.txt，
并将其添加到主CMakeLists.txt中，从而可以在自己的目录范围内处理该文件:

.. code-block:: bash

  add_subdirectory(src)

在src/CMakeLists.txt中，可以定义可执行目标，并列出从build.log中获取所有源码:

.. code-block:: bash

  add_executable(vim
      arabic.c 
      beval.c 
      buffer.c 
      blowfish.c 
      crypt.c 
      crypt_zip.c 
      dict.c diff.c 
      digraph.c 
      edit.c 
      eval.c 
      evalfunc.c 
      ex_cmds.c 
      ex_cmds2.c 
      ex_docmd.c 
      ex_eval.c 
      ex_getln.c 
      farsi.c 
      fileio.c 
      fold.c 
      getchar.c 
      hardcopy.c 
      hashtab.c 
      if_cscope.c 
      if_xcmdsrv.c 
      list.c 
      mark.c 
      memline.c menu.c 
      misc1.c misc2.c 
      move.c mbyte.c 
      normal.c 
      ops.c option.c 
      os_unix.c auto/pathdef.c 
      popupmnu.c pty.c 
      quickfix.c regexp.c 
      screen.c search.c sha256.c 
      spell.c spellfile.c 
      syntax.c tag.c term.c 
      terminal.c ui.c 
      undo.c 
      userfunc.c window.c 
      libvterm/src/encoding.c 
      libvterm/src/keyboard.c 
      libvterm/src/mouse.c 
      libvterm/src/parser.c 
      libvterm/src/pen.c 
      libvterm/src/screen.c 
      libvterm/src/state.c 
      libvterm/src/unicode.c 
      libvterm/src/vterm.c 
      netbeans.c channel.c 
      charset.c 
      json.c 
      main.c 
      memfile.c 
      message.c 
      version.c
  )

这是一个开始。这种情况下，代码甚至不会配置，因为源列表包含生成的文件。讨论生成文件和链接依赖项之前，我们把这一长列表拆分一下，以限制目标依赖项的范围，
并使项目更易于管理。如果我们将它们分组到目标，这将使CMake更容易地找到源文件依赖项，并避免很长的链接行。

对于Vim示例，我们可以进一步了解来自src/Makefile和src/configure.ac的源码文件进行分组。这些文件中，大多数源文件都是必需的。
有些源文件是可选的(netbeans.c应该只在ENABLE_NETBEANS打开时构建，而channel.c应该只在ENABLE_CHANNEL打开时构建)。
此外，我们可以将所有源代码分组到src/libvterm/下，并使用ENABLE_TERMINAL可选地编译它们。

这样，我们将CMake结构重组，构成如下的树结构：

.. code-block:: bash

  .
  ├── CMakeLists.txt
  └── src
      ├── CMakeLists.txt
      └── libvterm
          └── CMakeLists.txt
          
顶层文件使用add_subdirectory(src)添加src/CMakeLists.txt。src/CMakeLists.txt文件包含三个目标(一个可执行文件和两个库)，
每个目标都带有编译定义和包含目录。首先定义可执行文件：

.. code-block:: bash

  add_executable(vim
    main.c
    )
  target_compile_definitions(vim
    PRIVATE
        "HAVE_CONFIG_H"
    )

然后，定义一些需要源码文件的目标:

.. code-block:: bash

  add_library(basic_sources "")
  target_sources(basic_sources
    PRIVATE
      arabic.c beval.c blowfish.c buffer.c charset.c
      crypt.c crypt_zip.c dict.c diff.c digraph.c
      edit.c eval.c evalfunc.c ex_cmds.c ex_cmds2.c
      ex_docmd.c ex_eval.c ex_getln.c farsi.c fileio.c
      fold.c getchar.c hardcopy.c hashtab.c if_cscope.c
      if_xcmdsrv.c json.c list.c main.c mark.c
      memfile.c memline.c menu.c message.c misc1.c
      misc2.c move.c mbyte.c normal.c ops.c
      option.c os_unix.c auto/pathdef.c popupmnu.c pty.c
      quickfix.c regexp.c screen.c search.c sha256.c
      spell.c spellfile.c syntax.c tag.c term.c
      terminal.c ui.c undo.c userfunc.c version.c
      window.c
    )
  target_include_directories(basic_sources
    PRIVATE
      ${CMAKE_CURRENT_LIST_DIR}/proto
      ${CMAKE_CURRENT_LIST_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
    )
  target_compile_definitions(basic_sources
    PRIVATE
        "HAVE_CONFIG_H"
    )
  target_link_libraries(vim
    PUBLIC
        basic_sources
    )

然后，定义一些可选源码文件的目标:

.. code-block:: bash

  add_library(extra_sources "")
  if(ENABLE_NETBEANS)
    target_sources(extra_sources
      PRIVATE
          netbeans.c
      )
  endif()
  if(ENABLE_CHANNEL)
    target_sources(extra_sources
      PRIVATE
          channel.c
      )
  endif()
  target_include_directories(extra_sources
    PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/proto
      ${CMAKE_CURRENT_BINARY_DIR}
    )
  target_compile_definitions(extra_sources
    PRIVATE
        "HAVE_CONFIG_H"
    )
  target_link_libraries(vim
    PUBLIC
        extra_sources
    )

使用以下代码，对连接src/libvterm/子目录进行选择:

.. code-block:: bash

  if(ENABLE_TERMINAL)
    add_subdirectory(libvterm)
    target_link_libraries(vim
      PUBLIC
          libvterm
      )
  endif()

对应的src/libvterm/CMakeLists.txt包含以下内容:

.. code-block:: bash

  add_library(libvterm "")
  target_sources(libvterm
    PRIVATE
      src/encoding.c
      src/keyboard.c
      src/mouse.c
      src/parser.c
      src/pen.c
      src/screen.c
      src/state.c
      src/unicode.c
      src/vterm.c
    )
  target_include_directories(libvterm
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
    )
  target_compile_definitions(libvterm
    PRIVATE
      "HAVE_CONFIG_H"
      "INLINE="
      "VSNPRINTF=vim_vsnprintf"
      "IS_COMBINING_FUNCTION=utf_iscomposing_uint"
      "WCWIDTH_FUNCTION=utf_uint2cells"
    )

我们已经从build.log中获取了编译信息。树结构的优点是，目标的定义靠近源的位置。如果我们决定重构代码并重命名或移动目录，描述目标的CMake文件就会随着源文件一起移动。

我们的示例代码还没有配置(除非在成功的Autotools构建之后尝试配置)，现在来试试:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  -- The C compiler identification is GNU 8.2.0
  -- Check for working C compiler: /usr/bin/cc
  -- Check for working C compiler: /usr/bin/cc -- works
  -- Detecting C compiler ABI info
  -- Detecting C compiler ABI info - done
  -- Detecting C compile features
  -- Detecting C compile features - done
  -- Configuring done
  CMake Error at src/CMakeLists.txt:12 (add_library):
  Cannot find source file:
  auto/pathdef.c
  Tried extensions .c .C .c++ .cc .cpp .cxx .cu .m .M .mm .h .hh .h++ .hm
  .hpp .hxx .in .txx

这里需要生成auto/pathdef.c(和其他文件)，我们将在下一节中考虑这些文件。

15.2 生成文件并编写平台检查
-----------------------------

对于Vim示例，我们需要在配置时生成三个文件，src/auto/pathdef.c、src/auto/config.h和src/auto/osdef.h:

* pathdef.c：记录安装路径、编译/链接标志、当前用户和主机名
* config.h：编译系统的环境
* osdef.h：由src/osdef.sh生成的文件

这种情况相当普遍。需要CMake配置文件，配置时执行一个脚本，执行许多平台检查命令，来生成config.h。特别是，对于那些可移植的项目，平台检查非常普遍。

在原始目录树中，文件在src文件夹下生成。而我们将使用不同的方法：这些文件会生成在build目录中。这样做的原因是生成的文件通常依赖于所选择的选项、
编译器或构建类型，我们希望保持同一个源，可以适配多个构建。要在build目录中启用生成，我们必须对生成文件的脚本进行改动。

**构造文件**

我们将把与生成文件相关的函数集中放在src/autogenerate.cmake中。在定义可执行目标之前，在src/CMakeLists.txt中调用这些函数:

.. code-block:: bash

  # generate config.h, pathdef.c, and osdef.h
  include(autogenerate.cmake)
  generate_config_h()
  generate_pathdef_c()
  generate_osdef_h()
  add_executable(vim
        main.c
    )
  # ...

src/autogenerate.cmake中包含了其他检测头文件、函数和库等几个函数:

.. code-block:: bash

  include(CheckTypeSize)
  include(CheckFunctionExists)
  include(CheckIncludeFiles)
  include(CheckLibraryExists)
  include(CheckCSourceCompiles)
  function(generate_config_h)
      # ... to be written
  endfunction()
  function(generate_pathdef_c)
      # ... to be written
  endfunction()
  function(generate_osdef_h)
      # ... to be written
  endfunction()

我们选择了一些用于生成文件的函数，而不是用宏或“裸”CMake代码。在前几章中讨论过的，这是避免了一些问题：

* 避免多次生成文件，以防多次包含模块。我们可以使用一个包含保护来防止意外地多次运行代码。
* 保证了对函数中变量范围的完全控制。这避免了这些定义溢出，从而出现变量污染的情况。

**根据系统配置预处理宏定义**

config.h文件以src/config.h.in为目标所生成的，其中包含根据系统功能配置的预处理标志:

.. code-block:: bash

  /* Define if we have EBCDIC code */
  #undef EBCDIC
  /* Define unless no X support found */
  #undef HAVE_X11
  /* Define when terminfo support found */
  #undef TERMINFO
  /* Define when termcap.h contains ospeed */
  #undef HAVE_OSPEED
  /* ... */

生成的src/config.h示例类似如下情况(定义可以根据环境的不同而不同):

.. code-block:: bash

  /* Define if we have EBCDIC code */
  /* #undef EBCDIC */
  /* Define unless no X support found */
  #define HAVE_X11 1
  /* Define when terminfo support found */
  #define TERMINFO 1
  /* Define when termcap.h contains ospeed */
  /* #undef HAVE_OSPEED */
  /* ... */

这个页面是一个很好的平台检查示例: https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/How-To-Write-Platform-Checks

在src/configure.ac中，我们可以检查需要执行哪些平台检查，从而来设置相应的预处理定义。

我们将使用#cmakedefine(https://cmake.org/cmake/help/v3.5/command/configure_file.html?highlight=cmakedefine )
为了确保不破坏现有的Autotools构建，我们将复制config.h.in为config.h.cmake.in，并将所有#undef SOME_DEFINITION更改为
#cmakedefine SOME_DEFINITION @SOME_DEFINITION@。

在generate_config_h函数中，先定义两个变量：

.. code-block:: bash

  set(TERMINFO 1)
  set(UNIX 1)
  # this is hardcoded to keep the discussion in the book chapter
  # which describes the migration to CMake simpler
  set(TIME_WITH_SYS_TIME 1)
  set(RETSIGTYPE void)
  set(SIGRETURN return)
  find_package(X11)
  set(HAVE_X11 ${X11_FOUND})

然后，我们执行几个类型检查:

.. code-block:: bash

  check_type_size("int" VIM_SIZEOF_INT)
  check_type_size("long" VIM_SIZEOF_LONG)
  check_type_size("time_t" SIZEOF_TIME_T)
  check_type_size("off_t" SIZEOF_OFF_T)

然后，我们对函数进行循环，检查系统是否能够解析:

.. code-block:: bash

  foreach(
    _function IN ITEMS
    fchdir fchown fchmod fsync getcwd getpseudotty
    getpwent getpwnam getpwuid getrlimit gettimeofday getwd lstat
    memset mkdtemp nanosleep opendir putenv qsort readlink select setenv
    getpgid setpgid setsid sigaltstack sigstack sigset sigsetjmp sigaction
    sigprocmask sigvec strcasecmp strerror strftime stricmp strncasecmp
    strnicmp strpbrk strtol towlower towupper iswupper
    usleep utime utimes mblen ftruncate
    )
    string(TOUPPER "${_function}" _function_uppercase)
    check_function_exists(${_function} HAVE_${_function_uppercase})
  endforeach()

验证库是否包含特定函数:

.. code-block:: bash

  check_library_exists(tinfo tgetent "" HAVE_TGETENT)
  if(NOT HAVE_TGETENT)
      message(FATAL_ERROR "Could not find the tgetent() function. You need to install a terminal library; for example ncurses.")
  endif()

然后，我们循环头文件，检查它们是否可用:

.. code-block:: bash

  foreach(
    _header IN ITEMS
    setjmp.h dirent.h
    stdint.h stdlib.h string.h
    sys/select.h sys/utsname.h termcap.h fcntl.h
    sgtty.h sys/ioctl.h sys/time.h sys/types.h
    termio.h iconv.h inttypes.h langinfo.h math.h
    unistd.h stropts.h errno.h sys/resource.h
    sys/systeminfo.h locale.h sys/stream.h termios.h
    libc.h sys/statfs.h poll.h sys/poll.h pwd.h
    utime.h sys/param.h libintl.h libgen.h
    util/debug.h util/msg18n.h frame.h sys/acl.h
    sys/access.h sys/sysinfo.h wchar.h wctype.h
    )
    string(TOUPPER "${_header}" _header_uppercase)
    string(REPLACE "/" "_" _header_normalized "${_header_uppercase}")
    string(REPLACE "." "_" _header_normalized "${_header_normalized}")
    check_include_files(${_header} HAVE_${_header_normalized})
  endforeach()

然后，我们将CMake选项从转换为预处理定义:

.. code-block:: bash

  string(TOUPPER "${FEATURES}" _features_upper)
  set(FEAT_${_features_upper} 1)
  set(FEAT_NETBEANS_INTG ${ENABLE_NETBEANS})
  set(FEAT_JOB_CHANNEL ${ENABLE_CHANNEL})
  set(FEAT_TERMINAL ${ENABLE_TERMINAL})

最后，我们检查是否能够编译一个特定的代码片段:

.. code-block:: bash

  check_c_source_compiles(
    "
    #include <sys/types.h>
    #include <sys/stat.h>
    int
    main ()
    {
      struct stat st;
      int n;
      stat(\"/\", &st);
      n = (int)st.st_blksize;
      ;
      return 0;
    }
    "
    HAVE_ST_BLKSIZE
    )

然后，使用定义的变量配置src/config.h.cmake.in生成config.h，其中包含generate_config_h函数：

.. code-block:: bash

  configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/config.h.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/auto/config.h
    @ONLY
    )

**使用路径和编译器标志配置文件**

从src/pathdef.c.in生成pathdef.c:

.. code-block:: bash

  #include "vim.h"
  char_u *default_vim_dir = (char_u *)"@_default_vim_dir@";
  char_u *default_vimruntime_dir = (char_u *)"@_default_vimruntime_dir@";
  char_u *all_cflags = (char_u *)"@_all_cflags@";
  char_u *all_lflags = (char_u *)"@_all_lflags@";
  char_u *compiled_user = (char_u *)"@_compiled_user@";
  char_u *compiled_sys = (char_u *)"@_compiled_sys@";

generate_pathdef_c函数在src/pathdef.c.in进行配置。为了简单起见，我们省略了链接标志:

.. code-block:: bash

  function(generate_pathdef_c)
    set(_default_vim_dir ${CMAKE_INSTALL_PREFIX})
    set(_default_vimruntime_dir ${_default_vim_dir})
    set(_all_cflags "${CMAKE_C_COMPILER} ${CMAKE_C_FLAGS}")
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(_all_cflags "${_all_cflags} ${CMAKE_C_FLAGS_RELEASE}")
    else()
        set(_all_cflags "${_all_cflags} ${CMAKE_C_FLAGS_DEBUG}")
    endif()
    # it would require a bit more work and execute commands at build time
    # to get the link line into the binary
    set(_all_lflags "undefined")
    if(WIN32)
        set(_compiled_user $ENV{USERNAME})
    else()
        set(_compiled_user $ENV{USER})
    endif()
    cmake_host_system_information(RESULT _compiled_sys QUERY HOSTNAME)
    configure_file(
      ${CMAKE_CURRENT_LIST_DIR}/pathdef.c.in
      ${CMAKE_CURRENT_BINARY_DIR}/auto/pathdef.c
      @ONLY
      )
  endfunction()

**配置时执行shell脚本**

最后，我们使用以下函数生成osdef.h:

.. code-block:: bash

  function(generate_osdef_h)
    find_program(BASH_EXECUTABLE bash)
    execute_process(
      COMMAND
      ${BASH_EXECUTABLE} osdef.sh ${CMAKE_CURRENT_BINARY_DIR}
      WORKING_DIRECTORY
      ${CMAKE_CURRENT_LIST_DIR}
      )
  endfunction()

为了在${CMAKE_CURRENT_BINARY_DIR}/src/auto而不是src/auto中生成osdef.h，我们必须调整osdef.sh以接受${CMAKE_CURRENT_BINARY_DIR}作为命令行参数。

osdef.sh中，我们会检查是否给定了这个参数:

.. code-block:: bash

  if [ $# -eq 0 ]
  then
    # there are no arguments
    # assume the target directory is current directory
    target_directory=$PWD
  else
    # target directory is provided as argument
    target_directory=$1
  fi

然后，生成${target_directory}/auto/osdef.h。为此，我们还必须在osdef.sh中调整以下行:

.. code-block:: bash

  $CC -I. -I$srcdir -
  I${target_directory} -E osdef0.c >osdef0.cc


15.3 检测所需的链接和依赖关系
-----------------------------

现在已经生成了所有文件，让我们重新构建。我们应该能够配置和编译源代码，不过不能链接:

.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake ..
  $ cmake --build .
  ...
  Scanning dependencies of target vim
  [ 98%] Building C object src/CMakeFiles/vim.dir/main.c.o
  [100%] Linking C executable ../bin/vim
  ../lib64/libbasic_sources.a(term.c.o): In function `set_shellsize.part.12':
  term.c:(.text+0x2bd): undefined reference to `tputs'
  ../lib64/libbasic_sources.a(term.c.o): In function `getlinecol':
  term.c:(.text+0x902): undefined reference to `tgetent'
  term.c:(.text+0x915): undefined reference to `tgetent'
  term.c:(.text+0x935): undefined reference to `tgetnum'
  term.c:(.text+0x948): undefined reference to `tgetnum'
  ... many other undefined references ...

同样，可以从Autotools编译中获取日志文件，特别是链接行，通过在src/CMakeLists.txt中添加以下代码来解决缺少的依赖关系:

.. code-block:: bash

  # find X11 and link to it
  find_package(X11 REQUIRED)
  if(X11_FOUND)
    target_link_libraries(vim
      PUBLIC
          ${X11_LIBRARIES}
      )
  endif()
  # a couple of more system libraries that the code requires
  foreach(_library IN ITEMS Xt SM m tinfo acl gpm dl)
    find_library(_${_library}_found ${_library} REQUIRED)
    if(_${_library}_found)
      target_link_libraries(vim
        PUBLIC
          ${_library}
        )
    endif()
  endforeach()

我们可以添加一个库的依赖目标，并且不需要构建，以及不需要将库目标放在一个列表变量中，否则将破坏CMake代码的自变量，特别是对于较大的项目而言。

修改之后，编译和链接:

.. code-block:: bash

$ cmake --build .
  ...
  Scanning dependencies of target vim
  [ 98%] Building C object src/CMakeFiles/vim.dir/main.c.o
  [100%] Linking C executable ../bin/vim
  [100%] Built target vim

现在，我们可以执行编译后的二进制文件，我们新编译的Vim就可使用了!


15.4 复制编译标志
-----------------------------

现在，让我们尝试调整编译器标志来进行引用构建。

**定义编译器标志**

目前为止，我们还没有定义任何自定义编译器标志，参考Autotools构建中，代码是使用的编译标志有-g -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=1 -O2，
这些标示都是GNU C编译器可以识别的。

我们的第一个定义如下:

.. code-block:: bash

  if(CMAKE_C_COMPILER_ID MATCHES GNU)
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=1 -O2")
  endif() 

并且，在生成源文件之前，我们将把这段代码放在src/CMakeLists.txt的顶部(因为pathdef.c有使用到${CMAKE_C_FLAGS}):

.. code-block:: bash

  # <- we will define flags right here
  include(autogenerate.cmake)
  generate_config_h()
  generate_pathdef_c()
  generate_osdef_h()

编译器标志定义的一个小修改是将-O2定义为Release配置标志，并关闭Debug的配置:

.. code-block:: bash

  if(CMAKE_C_COMPILER_ID MATCHES GNU)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -U_FORTIFY_SOURCE
    -D_FORTIFY_SOURCE=1")
    set(CMAKE_C_FLAGS_RELEASE "-O2")
    set(CMAKE_C_FLAGS_DEBUG "-O0")
  endif()

请使用make VERBOSE=1验证，构建是否使用了预期的标志。

**编译器标志的作用域**

在这个特殊的示例项目中，所有源文件都使用相同的编译标志。对于其他项目，我们可能不希望像上面那样全局定义编译标志，
而是使用target_compile_options为每个目标分别定义编译标志。这样做的好处是更灵活、范围更小。在我们的例子中，这能减少不必要的代码复制。


15.5 移植测试
-----------------------------

现在，来讨论如何将测试从引用构建移植到CMake。

**准备工作**

如果移植的项目包含测试目标，或任何形式的自动化测试，以及测试脚本。第一步，运行传统的测试步骤，并记录所使用的命令。
对于Vim项目，可以从src/testdir/Makefile开始。在src/testdir/Makefile和测试脚本中的一些对于测试的定义，
我们将在src/testdir/CMakeLists.txt中进行相应的定义。所以，我们必须在src/CMakeLists.txt中引用它:

.. code-block:: bash

  add_subdirectory(testdir)

处理src/CMakeLists.txt之前，我们还应该在主CMakeLists.txt中启用测试:

.. code-block:: bash

  # enable the test target
  enable_testing()
  # process src/CMakeLists.txt in its own scope
  add_subdirectory(src)

目前为止，使用add_test填充src/testdir/CMakeLists.txt之前，测试目标为空。在add_test中指定要运行的测试名称和命令。
该命令可以用任何语言编写。CMake的关键部分是，如果测试成功，脚本返回零；如果测试失败，脚本返回非零。对于Vim，我们需要多步骤测试，这将在下一节中讨论。

**实现多步测试**

在src/testdir/Makefile的目标表明，Vim代码运行测试多步测试：

* Vim脚本可执行测试流程，产生一个输出文件
* 输出文件是与参考文件进行比，,如果这些文件相同，测试成功
* 删除临时文件

由于add_test只能执行一个命令，因此无法以可移植的方式将其放到单个add_test中。一种解决方案是在Python脚本中定义测试步骤，
并使用一些参数执行Python脚本。这里提供的另一种选择，也是跨平台的，在单独的CMake脚本中定义测试步骤，
并使用add_test执行这个脚本。我们将在src/testdir/test.cmake中定义测试步骤:

.. code-block:: bash

  function(execute_test _vim_executable _working_dir _test_script)
    # generates test.out
    execute_process(
      COMMAND ${_vim_executable} -f -u unix.vim -U NONE --noplugin --not-a-term -s dotest.in ${_test_script}.in
      WORKING_DIRECTORY ${_working_dir}
      )
    # compares test*.ok and test.out
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E compare_files ${_test_script}.ok test.out
      WORKING_DIRECTORY ${_working_dir}
      RESULT_VARIABLE files_differ
      OUTPUT_QUIET
      ERROR_QUIET
      )
    # removes leftovers
    file(REMOVE ${_working_dir}/Xdotest)
    # we let the test fail if the files differ
    if(files_differ)
        message(SEND_ERROR "test ${_test_script} failed")
    endif()
  endfunction()
  execute_test(${VIM_EXECUTABLE} ${WORKING_DIR} ${TEST_SCRIPT})

同样，我们选择函数而不是宏，为的是使得变量不会超出函数作用域。它将调用execute_test函数，处理这个脚本。
但是，我们必须确保${VIM_EXECUTABLE}、${WORKING_DIR}和${TEST_SCRIPT}是在外部定义。src/testdir/CMakeLists.txt中定义:

.. code-block:: bash

  add_test(
    NAME
        test1
    COMMAND
      ${CMAKE_COMMAND} -D VIM_EXECUTABLE=$<TARGET_FILE:vim>
      -D WORKING_DIR=${CMAKE_CURRENT_LIST_DIR}
      -D TEST_SCRIPT=test1
      -P ${CMAKE_CURRENT_LIST_DIR}/test.cmake
    WORKING_DIRECTORY
        ${PROJECT_BINARY_DIR}
    )

Vim项目有很多测试，但是在这个例子中，我们只移植了一个(test1)。

**测试建议**

对于移植测试，我们可以给出至少两个建议。

* 要确保测试并不总是报告成功，如果破坏了代码或修改了验证数据，请验证测试是否失败。
* 添加测试的成本估算，以便在并行运行时，首先启动较长的测试，以最小化总测试时间。


15.6 移植安装目标
-----------------------------

现在可以配置、编译、链接和测试代码，但是没有测试安装目标。我们将在本节中添加这个目标。

Autotools的构建和安装方式:


.. code-block:: bash

  $ ./configure --prefix=/some/install/path
  $ make
  $ make install

以下是CMake的方式：


.. code-block:: bash

  $ mkdir -p build
  $ cd build
  $ cmake -D CMAKE_INSTALL_PREFIX=/some/install/path ..
  $ cmake --build .
  $ cmake --build . --target install

要添加安装目标，需要在src/CMakeLists.txt中添加以下代码:


.. code-block:: bash

  install(
    TARGETS
        vim
    RUNTIME DESTINATION
        ${CMAKE_INSTALL_BINDIR}
    )

本例中，只安装了可执行文件。Vim项目需要安装大量文件(符号链接和文档文件)，为了使本节易于理解，我们就没有迁移示例中所有的安装目标。
对于自己的项目而言，应该验证安装步骤的结果是否匹配之前构建框架的安装目标。


15.7 进一步迁移的措施
-----------------------------

成功地移植到CMake之后，下一步应该本地化目标和变量的范围：考虑将选项、目标和变量移到更靠近使用和修改它们的地方。避免全局变量，
因为它们将按CMake命令顺序进行创建，而这个顺序可能不明显，从而会导致CMake代码变得混乱。强制分离变量范围的一种方法是将较大的项目划分为CMake项目，
这些项目使用超构建块组成。从而，可考虑将大型CMakeLists.txt文件分割成更小的模块。

接下来的步骤，可以是在其他平台和操作系统上进行配置和编译，以便增强CMake代码的鲁棒性，使其更具可移植性。

最后，将项目迁移到新的构建框架时，开发人员社区也需要去适应。为了帮助您的同事进行培训、文档编制和代码评审。将代码移植到CMake中最困难的部分，
可能是改变相关人员的使用习惯。


15.8 项目转换为CMake的常见问题
-------------------------------

我们总结一下，在这一章中所所学到的知识。

**代码修改总结**

在本章中，讨论了如何将项目移植到CMake进行构建。我们以Vim项目为例，添加了以下文件:

.. code-block:: bash

  .
  ├── CMakeLists.txt
  └── src
      ├── autogenerate.cmake
      ├── CMakeLists.txt
      ├── config.h.cmake.in
      ├── libvterm
      │    └── CMakeLists.txt
      ├── pathdef.c.in
      └── testdir
          ├── CMakeLists.txt
          └── test.cmake

可以在线查看修改： https://github.com/dev-cafe/vim/compare/b476cb7...cmake-support

为了简单起见，我们省略了许多选项和调整，并将重点放在最重要的步骤上。

**常见问题**

在结束讨论之前，我们想指出一些迁移到CMake时常见的问题。

* 全局变量代码异味：这点适用于任何编程语言，CMake也不例外。跨CMake文件的变量，特别是从子到父CMakeLists.txt文件的“向上”传递的变量，这是明显的“异味代码”。
  通常，会有一种更好的方法来传输依赖关系。理想情况下，依赖项应该通过目标导入。与其将库列表组装成一个变量并在文件之间携带该变量，
  不如逐个链接到定义库的地方。不是将源文件组装成变量，而是使用target_sources添加源文件。当链接到库时，在可用时使用导入的目标，而不是变量。
* 最小化顺序的影响：CMake不是一种声明性语言，但是也不应该使用命令式范式进行处理。执行严格命令的代码往往是脆弱的，
  这也与变量有关(见上一段)。一些语句和模块的顺序是必要的，但是为了实现健壮的CMake框架，我们应该避免不必要的顺序强制。
  应该多使用target_sources、target_compile_definition、target_include_directory和target_link_libraries。
  避免使用全局范围语句，如add_definition、include_directory和link_libraries，从而避免定义全局编译标志。如果可能，为每个目标定义编译标志。
* 不在build目录之外生成文件：强烈建议不要将生成的文件放在构建目录之外。原因是生成的文件通常依赖于所选择的选项、编译器或构建类型。
  如果写入原目录树，我们就放弃了用同一套源码维护多个构建的可能性，并且会使构建步骤的重现复杂化。
* 尽可能使用函数，而不是宏：它们的作用范围不同，功能范围也有限定。所有变量修改都需要显式标记，这也向读者展示了重新定义的变量。
  如果可以最好使用函数，必要时再使用宏。
* 避免shell命令：Shell可能不能移植到其他平台(如Windows)。可以使用CMake中的命令或函数。如果没有可用的CMake等效函数，请考虑调用Python脚本。
* Fortran中，注意后缀：需要预处理的Fortran源文件是大写的.F90后缀。无预处理的源文件应该以.f90为后缀。
* 避免显式路径：这条建议在定义目标和引用文件时都适用。当引用当前路径时，可使用CMAKE_CURRENT_LIST_DIR。
  这样做的好处是，当移动或重命名一个目录时，构建不会出问题。
* 不应该在函数调用中进行模块包含：将CMake代码模块化是一个很好的策略，但是包含模块不应该执行CMake代码。
  相反，将CMake代码封装到函数和宏中，并在包含模块之后显式地调用这些函数和宏。当意外地多次包含模块时，这条建议可以防止意外的副作用，
  并使执行CMake代码模块的操作更易读。
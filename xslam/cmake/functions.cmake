###################################################################
#  Created by dyq on 3/24/21.
#
# 为众人抱薪者，不可使其冻毙于风雪
# 为自由开道者，不可令其困厄于荆棘
#
# 不因情势为难, 而存必死之心
# 不因身份尊贵, 而轻慢侮人
# 不因才智独到, 就私谋为众
# 不因将士用命, 就好大喜功
#
###################################################################/

include(CMakeParseArguments)

macro(_parse_arguments ARGS)
  set(OPTIONS)
  set(ONE_VALUE_ARG)
  set(MULTI_VALUE_ARGS SRCS)
  cmake_parse_arguments(ARG
    "${OPTIONS}" "${ONE_VALUE_ARG}" "${MULTI_VALUE_ARGS}" ${ARGS})
endmacro(_parse_arguments)

macro(_common_compile_stuff VISIBILITY)
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${PROJECT_CXX_FLAGS}")

  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

  target_include_directories(${NAME} PUBLIC ${PROJECT_NAME})
  target_link_libraries(${NAME} PUBLIC ${PROJECT_NAME})
endmacro(_common_compile_stuff)

function(project_test NAME ARG_SRC)
  add_executable(${NAME} ${ARG_SRC})
  _common_compile_stuff("PRIVATE")

  # Make sure that gmock always includes the correct gtest/gtest.h.
  target_include_directories("${NAME}" SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries("${NAME}" PUBLIC ${GMOCK_LIBRARIES})
  add_test(${NAME} ${NAME})
endfunction()

function(project_binary NAME)
  _parse_arguments("${ARGN}")

  add_executable(${NAME} ${ARG_SRCS})

  _common_compile_stuff("PRIVATE")

  install(TARGETS "${NAME}" RUNTIME DESTINATION bin)
endfunction()

# Create a variable 'VAR_NAME'='FLAG'. If VAR_NAME is already set, FLAG is
# appended.
function(project_add_flag VAR_NAME FLAG)
  if (${VAR_NAME})
    set(${VAR_NAME} "${${VAR_NAME}} ${FLAG}" PARENT_SCOPE)
  else()
    set(${VAR_NAME} "${FLAG}" PARENT_SCOPE)
  endif()
endfunction()

macro(project_initialize)
  if(PROJECT_CMAKE_DIR)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${PROJECT_CMAKE_DIR}/modules)
  else()
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
  endif()

  set(PROJECT_CXX_FLAGS "-pthread -std=c++11 -fPIC ${PROJECT_CXX_FLAGS}")
  add_definitions("-DENABLE_SSE")
  project_add_flag(PROJECT_CXX_FLAGS "-Wall")
  project_add_flag(PROJECT_CXX_FLAGS "-Wpedantic")
  project_add_flag(PROJECT_CXX_FLAGS "-mpopcnt")
  project_add_flag(CMAKE_CXX_FLAGS "${SSE_FLAGS} -msse4")

  # Turn some warnings into errors.
  project_add_flag(PROJECT_CXX_FLAGS "-Werror=format-security")
  project_add_flag(PROJECT_CXX_FLAGS "-Werror=missing-braces")
  project_add_flag(PROJECT_CXX_FLAGS "-Werror=reorder")
  project_add_flag(PROJECT_CXX_FLAGS "-Werror=return-type")
  project_add_flag(PROJECT_CXX_FLAGS "-Werror=switch")
  project_add_flag(PROJECT_CXX_FLAGS "-Werror=uninitialized")


  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "AppleClang")
    project_add_flag(PROJECT_CXX_FLAGS "-Wthread-safety")
  endif()

  if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
  endif()

  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    project_add_flag(PROJECT_CXX_FLAGS "-O3 -DNDEBUG")
  elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    project_add_flag(PROJECT_CXX_FLAGS "-O3 -g -DNDEBUG")
  elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
      message(WARNING "Building in Debug mode, expect very slow performance.")
      project_add_flag(PROJECT_CXX_FLAGS "-g")
  # Support for Debian packaging CMAKE_BUILD_TYPE
  elseif(CMAKE_BUILD_TYPE STREQUAL "None")
    message(WARNING "Building with CMAKE_BUILD_TYPE None, "
        "please make sure you have set CFLAGS and CXXFLAGS according to your needs.")
  else()
    message(FATAL_ERROR "Unknown CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
  endif()

  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

  # Add a hook that reruns CMake when source files are added or removed.
  set(LIST_FILES_CMD "find ${PROJECT_SOURCE_DIR}/ -not -iwholename '*.git*' | sort | sed 's/^/#/'")
  set(FILES_LIST_PATH "${PROJECT_BINARY_DIR}/AllFiles.cmake")
  set(DETECT_CHANGES_CMD "bash" "-c" "${LIST_FILES_CMD} | diff -N -q ${FILES_LIST_PATH} - || ${LIST_FILES_CMD} > ${FILES_LIST_PATH}")
  add_custom_target(${PROJECT_NAME}_detect_changes ALL
    COMMAND ${DETECT_CHANGES_CMD}
    VERBATIM
  )
  if(NOT EXISTS ${FILES_LIST_PATH})
    execute_process(COMMAND ${DETECT_CHANGES_CMD})
  endif()
  include(${FILES_LIST_PATH})

endmacro()

macro(project_enable_testing)
  enable_testing()
  find_package(GMock REQUIRED)
endmacro()

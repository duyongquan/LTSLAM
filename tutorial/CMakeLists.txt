set(CMAKE_CXX_STANDARD 17)

# lib bin
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(BUILD_CPP_TEST  "Project build test" ON)
include("./cmake/functions.cmake")
project_initialize()
project_enable_testing()

file(GLOB_RECURSE COMMON_HDRS "common/*.h" "common/*.hpp")
file(GLOB_RECURSE COMMON_SRCS "common/*.cpp")
file(GLOB_RECURSE ALL_CPP_TESTS "cpp/*_test.cpp")

if (BUILD_CPP_TEST)
    foreach(ABS_FIL ${ALL_CPP_TESTS})
        file(RELATIVE_PATH REL_FIL ${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL})
        get_filename_component(DIR ${REL_FIL} DIRECTORY)
        get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
        string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
        project_cpp_test("${TEST_TARGET_NAME}" ${ABS_FIL})
        target_include_directories(${TEST_TARGET_NAME} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
        target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${GTEST_LIBRARIES})
    endforeach()
endif()
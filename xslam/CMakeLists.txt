cmake_minimum_required(VERSION 3.2)
project(xslam)

set(CMAKE_CXX_STANDARD 14)

# set version
set(SLAM_MAJOR_VERSION 0)
set(SLAM_MINOR_VERSION 0)
set(SLAM_PATCH_VERSION 1)
set(SLAM_VERSION ${SLAM_MAJOR_VERSION}.${SLAM_MINOR_VERSION}.${SLAM_PATCH_VERSION})
set(SLAM_SOVERSION ${SLAM_MAJOR_VERSION}.${SLAM_MINOR_VERSION})

# lib bin
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(BUILD_TEST  "Project build test" ON)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
project_initialize()
project_enable_testing()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)
find_package(G2O REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(CSparse)
find_package(DBoW3)
find_package(Protobuf 3.0.0 REQUIRED)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "xslam/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "xslam/*.cpp")
file(GLOB_RECURSE ALL_TESTS "xslam/*_test.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "xslam/*_main.cpp")
file(GLOB_RECURSE ALL_EXAMPLES "xslam/*_example.cpp")

list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXAMPLES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})

set(GLOG_LIBRARY glog)

#vlfeat 
add_subdirectory(libs)
set(VLFEAT_LIBRARY vlfeat)
set(VLFEAT_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/libs)
include_directories(${VLFEAT_INCLUDE_DIRS})

set(G2OLIBS
        g2o_core
        g2o_stuff
        g2o_solver_csparse
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        cholmod
        g2o_solver_cholmod
)

configure_file(
    ${PROJECT_SOURCE_DIR}/xslam/common/config.h.cmake
    ${PROJECT_BINARY_DIR}/xslam/common/config.h)

configure_file(
    ${PROJECT_SOURCE_DIR}/xslam/vins/common/config.h.cmake
    ${PROJECT_BINARY_DIR}/xslam/vins/common/config.h)

file(GLOB_RECURSE ALL_PROTOS "xslam/vins/*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS} ${ALL_LIBRARY_HPPS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC ${G2OLIBS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${CSPARSE_LIB})
target_link_libraries(${PROJECT_NAME} PUBLIC ${DBOW3_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} PUBLIC rt ${VLFEAT_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY})


target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Pangolin_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Sophus_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OpenCV_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${G2O_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CSPARSE_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${DBOW3_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${VLFEAT_INCLUDE_DIRS}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${LUA_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${PROTOBUF_INCLUDE_DIR}")


if (BUILD_TEST)
    foreach(ABS_FIL ${ALL_TESTS})
        file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
        get_filename_component(DIR ${REL_FIL} DIRECTORY)
        get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
        string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
        project_test("${TEST_TARGET_NAME}" ${ABS_FIL})
        target_include_directories(${TEST_TARGET_NAME} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
        target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${GTEST_LIBRARIES})
    endforeach()
endif()


target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

### Example
project_binary(glog_example
    SRCS
        xslam/examples/glog_example.cpp
)

project_binary(gflag_example
    SRCS
        xslam/examples/gflag_example.cpp
)

# VINS-Mono
project_binary(vins
    SRCS
        xslam/vins/vins_main.cpp
)

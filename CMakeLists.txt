cmake_minimum_required(VERSION 3.1.0)
project(ltslam)

set(CMAKE_CXX_STANDARD 17)

# catkin sets this to "on" by default, which causes linking problems
set(BUILD_SHARED_LIBS OFF)

# find GMock globally from here, or else it will complains about duplicated GTest/GMock targets
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/xslam/cmake/modules/")
find_package(GMock REQUIRED)

# use configuration files from the source location instead of the install location
set(XSLAM_CONFIGURATION_FILES_DIRECTORY "${CMAKE_SOURCE_DIR}/xslam/configuration_files"
  CACHE PATH ".lua config file directory")

# add the libxslam directory
add_subdirectory(xslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# Only build the documentation if we can find Sphinx.
find_package(Sphinx)
if(SPHINX_FOUND)
    add_subdirectory(docs)
endif()

# add catkin packages
add_subdirectory(xslam_ros/xslam_ros)


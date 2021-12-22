cmake_minimum_required(VERSION 3.1.0)
project(xslam_tutorial)
set(CMAKE_CXX_STANDARD 11)

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

# add catkin packages
add_subdirectory(xslam_ros/xslam_ros)
add_subdirectory(xslam_ros/docs)


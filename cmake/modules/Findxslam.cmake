set(XSLAM_CMAKE_DIR "${xslam_SOURCE_DIR}/cmake")

# this checks if xslam_SOURCE_DIR is set, and complains if not, indicating that libxslam has
# not been properly added to the build tree
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(xslam DEFAULT_MSG xslam_SOURCE_DIR)

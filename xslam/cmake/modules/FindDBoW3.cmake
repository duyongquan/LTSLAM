MESSAGE(STATUS "Looking for DBoW3...")

find_path(DBOW3_INCLUDE_DIR NAMES DBoW3.h HINTS /usr/local/include/DBoW3)
find_library(DBOW3_LIBRARY NAMES DBoW3 HINTS /usr/local/lib/DBoW3)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DBoW3 DEFAULT_MSG
    DBOW3_INCLUDE_DIR
    DBOW3_LIBRARY
)

mark_as_advanced(
    DBOW3_INCLUDE_DIR
    DBOW3_LIBRARY
)
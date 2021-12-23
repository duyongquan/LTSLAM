cmake_minimum_required (VERSION 2.8.7)

project(xslam_docs)

find_package(Sphinx)
if(SPHINX_FOUND)
  set(OUTPUT_DIR "${PROJECT_BINARY_DIR}/docs/html")
  add_custom_target(build_doc ALL
      ${SPHINX_EXECUTABLE} -b html
      ${CMAKE_CURRENT_SOURCE_DIR}/source
      ${CMAKE_CURRENT_BINARY_DIR}/html
      COMMENT "Building documentation."
  )
endif()

find_package(Sphinx REQUIRED)

# HTML output directory
set(SPHINX_HTML_DIR "${XSLAM_BINARY_DIR}/docs/html")

# Install documentation
install(DIRECTORY ${SPHINX_HTML_DIR}
        DESTINATION "${XSLAM_DOCS_INSTALL_DIR}"
        COMPONENT Doc
        PATTERN "${SPHINX_HTML_DIR}/*")

# Building using 'make_docs.py' python script
add_custom_target(xslam_docs ALL
                  python
                  "${XSLAM_SOURCE_DIR}/scripts/make_docs.py"
                  "${XSLAM_SOURCE_DIR}"
                  "${XSLAM_SOURCE_DIR}/docs"
                  "${SPHINX_EXECUTABLE}"
                  COMMENT "Building HTML documentation with Sphinx")

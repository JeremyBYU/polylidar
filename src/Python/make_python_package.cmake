# Clean up directory
file(REMOVE_RECURSE ${PYTHON_PACKAGE_DST_DIR})
file(MAKE_DIRECTORY ${PYTHON_PACKAGE_DST_DIR}/polylidar)

# Create python pacakge. It contains:
# 1) Pure-python code and misc files, copied from src/Python/package
# 2) The compiled python-C++ module, i.e. xxx.so (or the equivalents)
# 3) Configured files and supporting files

# 1) Pure-python code and misc files, copied from src/Python/package
file(COPY ${PYTHON_PACKAGE_SRC_DIR}/
     DESTINATION ${PYTHON_PACKAGE_DST_DIR}
)

# 2) The compiled python-C++ module, i.e. xxx.so (or the equivalents)
get_filename_component(PYTHON_COMPILED_MODULE_NAME ${PYTHON_COMPILED_MODULE_PATH} NAME)
file(COPY ${PYTHON_COMPILED_MODULE_PATH}
     DESTINATION ${PYTHON_PACKAGE_DST_DIR}/polylidar)

# 2.5) Copy the compiled module to the parent directory as well, only if this variable is set
if (ORIG_CMAKE_LIBRARY_OUTPUT_DIRECTORY)
     file(COPY ${PYTHON_COMPILED_MODULE_PATH}
          DESTINATION ${ORIG_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endif()


# 3) Configured files and supporting files
configure_file("${PYTHON_PACKAGE_SRC_DIR}/setup.py"
               "${PYTHON_PACKAGE_DST_DIR}/setup.py")
configure_file("${PYTHON_PACKAGE_SRC_DIR}/polylidar/__init__.py"
               "${PYTHON_PACKAGE_DST_DIR}/polylidar/__init__.py")




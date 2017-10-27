get_filename_component(CONFIG_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set(FREENECT_INCLUDE_DIRS "${CONFIG_DIR}/../../include/libfreenect")
set(FREENECT_RUNTIME_LIBRARY_DIRS "${CONFIG_DIR}/../../bin")
set(FREENECT_LIBRARY_DIRS "${CONFIG_DIR}/../../lib")
#/libfreenect.so.0.5.5
set(FREENECT_LIBRARIES "freenect")
set(FREENECT_VERSION "0.5.5")
set(FREENECT_FOUND "YES")
set(FREENECT_FOUND YES)

if(UNIX)
    find_path(LIBFREENECT_INCLUDE_DIRS "libfreenect/libfreenect.h"
                      PATHS "/usr/local/include"
                            "/usr/include")

    if(LIBFREENECT_DIR)
            mark_as_advanced(LIBFREENECT_INCLUDE_DIRS)

            # Libraries
            set(LIBFREENECT_RELEASE_NAME libfreenect.so)
            set(LIBFREENECT_DEBUG_NAME libfreenect.so)

            find_library(LIBFREENECT_LIBRARY
                               NAMES ${LIBFREENECT_RELEASE_NAME}
                               PATHS 	"/usr/lib/"
                                        "/usr/local/lib")

            mark_as_advanced(LIBFREENECT_LIBRARY)

            set(LIBFREENECT_LIBRARIES optimized ${LIBFREENECT_LIBRARY} debug ${LIBFREENECT_LIBRARY_DEBUG})
    endif(LIBFREENECT_INCLUDE_DIRS)
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(LIBFREENECT
FOUND_VAR LIBFREENECT_FOUND
REQUIRED_VARS LIBFREENECT_LIBRARIES LIBFREENECT_INCLUDE_DIRS
)

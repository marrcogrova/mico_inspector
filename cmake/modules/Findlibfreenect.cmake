###############################################################################
# Find CrossPlatform for freenect library
#
#     find_package(libfreenect)
#
# Variables defined by this module:
#
#  LIBFREENECT_FOUND                 True if librealsense was found
#  LIBFREENECT_INCLUDE_DIRS          The location(s) of RealSense SDK headers
#  LIBFREENECT_LIBRARY             Libraries needed to use RealSense SDK


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

    if(LIBFREENECT_INCLUDE_DIRS)
            mark_as_advanced(LIBFREENECT_INCLUDE_DIRS)

            # Libraries
            set(LIBFREENECT_NAME libfreenect.so)

            find_library(LIBFREENECT_LIBRARY
                               NAMES ${LIBFREENECT_NAME}
                               PATHS 	"/usr/lib/"
                                        "/usr/local/lib")

            mark_as_advanced(LIBFREENECT_LIBRARY)
            set(LIBFREENECT_LIBRARY ${LIBFREENECT_LIBRARY})
    endif(LIBFREENECT_INCLUDE_DIRS)
endif(UNIX)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(LIBFREENECT
FOUND_VAR LIBFREENECT_FOUND
REQUIRED_VARS LIBFREENECT_LIBRARY LIBFREENECT_INCLUDE_DIRS
)

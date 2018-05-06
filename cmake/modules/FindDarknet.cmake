###############################################################################
# Find Darknet manually installed
#
#     find_package(Darknet)
#
# Variables defined by this module:
#
#  Darknet_FOUND                 True if darknet was found
#  Darknet_INCLUDE_DIRS          The location(s) of darknet headers
#  Darknet_LIBRARIES             Libraries needed to use darknet


find_path(Darknet_DIR "include/darknet"
		  PATHS         "/usr/local"
                                "/usr")

if(Darknet_DIR)
        set(Darknet_INCLUDE_DIRS ${Darknet_DIR}/include)
	mark_as_advanced(Darknet_INCLUDE_DIRS)

	# Libraries
        set(Darknet_RELEASE_NAME libdarknet.so)

	find_library(Darknet_LIBRARY
			   NAMES ${Darknet_RELEASE_NAME}
			   PATHS 	"/usr/lib/"
					"/usr/local/lib")
	
	
	mark_as_advanced(Darknet_LIBRARY)
	set(Darknet_LIBRARIES optimized ${Darknet_LIBRARY})
	mark_as_advanced(Darknet_LIBRARIES)

endif(Darknet_DIR)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Darknet
	FOUND_VAR Darknet_FOUND
	REQUIRED_VARS Darknet_LIBRARIES Darknet_INCLUDE_DIRS
	)

##---------------------------------------------------------------------------------------------------------------------
##  RGBD_TOOLS
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
project(rgbd_tools VERSION 0.3 LANGUAGES C CXX)

##########################################
######		Configure project		######
##########################################
# ML modules
set(USE_DARKNET OFF)
set(USE_DARKNET_CL ON)

# SLAM modules
set(USE_G2O ON)
set(USE_DBOW2 ON)
set(USE_MODULE_SLAM ON)
set(USE_CATKIN OFF)

# Others
set(BUILD_TOOLS ON)
set(BUILD_EXAMPLES OFF)

#########################################
###### Loading 3rd party libraries.######
#########################################
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

list(APPEND RGBD_TOOLS_DEFINITIONS "")
set(RGBD_TOOLS_COMPILE_OPTIONS "-Wall")

#########################################
## Load librealsense
find_package(realsense2 HINTS "/usr/local/lib/cmake/realsense2")
if(${realsense2_FOUND})
	MESSAGE(STATUS "Found librealsense v2, compiling RealSense devices wrapper")
	list(APPEND RGBD_TOOLS_DEFINITIONS "ENABLE_LIBREALSENSE_V2")
else()
	find_package(librealsense)
	if(${LIBREALSENSE_FOUND})
		MESSAGE(STATUS "Found librealsense v1, compiling RealSense devices wrapper")
		list(APPEND RGBD_TOOLS_DEFINITIONS "ENABLE_LIBREALSENSE_V1")
	endif()
endif()



#########################################
## Load freenect
find_package(libfreenect)
if(${LIBFREENECT_FOUND})
	MESSAGE(STATUS "Found libfreenect, compiling kinect wrapper")
	list(APPEND RGBD_TOOLS_DEFINITIONS "ENABLE_LIBFREENECT")
endif()

if(${USE_MODULE_SLAM})
	if(${USE_DBOW2})
		find_package(DBoW2 REQUIRED)
		find_package(DLoopDetector REQUIRED)
		find_package(DLib REQUIRED)

		if((${DBoW2_FOUND} EQUAL "1") AND (${DLib_FOUND} EQUAL "1")AND (${DLoopDetector_FOUND} EQUAL "1"))
			list(APPEND RGBD_TOOLS_DEFINITIONS "USE_DBOW2")	
			SET(USE_DBOW2 OFF)
		endif()
	endif(${USE_DBOW2})

	# g2o dependencies
	if(${USE_G2O})
		set(G2O_ROOT /usr/local)
		FIND_PACKAGE(Eigen3 REQUIRED)
		FIND_PACKAGE(CSparse REQUIRED)
		FIND_PACKAGE(Cholmod REQUIRED)
		FIND_PACKAGE(OpenGL REQUIRED)
		FIND_PACKAGE(g2o REQUIRED HINTS "/usr/local/lib/cmake/g2o")
		if(${g2o_FOUND})
			list(APPEND RGBD_TOOLS_DEFINITIONS "USE_G2O")	
		else()
			message(Warning "NOT USING G2O")
			set(USE_G2O FALSE)
			set(g2o_FOUND FALSE)
		endif()
	endif(${USE_G2O})
endif(${USE_MODULE_SLAM})

#########################################
## Load opencv
find_package(OpenCV 3.4 REQUIRED core highgui imgproc imgcodecs features2d xfeatures2d calib3d video videoio)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

#########################################
## Load PCL
find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	# -march=native -msse4.2 -mfpmath=sse -DDISABLE_PCAP-DDISABLE_PNG-DDISABLE_LIBUSB_1_0
	list(APPEND RGBD_TOOLS_DEFINITIONS "ENABLE_PCL")
	list(APPEND RGBD_TOOLS_DEFINITIONS "DISABLE_PCAP-DDISABLE_PNG")
	list(APPEND RGBD_TOOLS_DEFINITIONS "DISABLE_LIBUSB_1_0")
	set(RGBD_TOOLS_COMPILE_OPTIONS "${RGBD_TOOLS_COMPILE_OPTIONS} -march=native -msse4.2 -mfpmath=sse -L/usr/lib/x86_64-linux-gnu")
	if(${PCL_VERSION} VERSION_GREATER 1.8)
		list(APPEND RGBD_TOOLS_DEFINITIONS "HAS_PCL_1_8")
	else()
		list(APPEND RGBD_TOOLS_DEFINITIONS "HAS_PCL_1_7")
	endif()
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

#########################################
## Load Artec SDK 
if(WIN32) # Look for a better way of doing this.
	if(${CMAKE_EXE_LINKER_FLAGS} MATCHES  "[/]([a-z])+[:]+x64")
		message(STATUS "Using ARTEC SDK with arch x64")
		file(GLOB_RECURSE ARTEC_LIBS "C:/Program Files/Artec/Artec 3D Scanning SDK/bin-x64/*.lib")
	elseif(${CMAKE_EXE_LINKER_FLAGS} MATCHES  "[/]([a-z])+[:]+x86")
	message(STATUS "Using ARTEC SDK with arch x86")
		file(GLOB_RECURSE ARTEC_LIBS "C:/Program Files/Artec/Artec 3D Scanning SDK/bin/*.lib")
	else()
		message(WARNING "Can't detect architecture configuration, assuming x86")
		file(GLOB_RECURSE ARTEC_LIBS "C:/Program Files/Artec/Artec 3D Scanning SDK/bin/*.lib")
	endif()
	
	set(ARTEC_INCLUDE_DIR "C:/Program Files/Artec/Artec 3D Scanning SDK/include")
	if(ARTEC_LIBS STREQUAL "")
		MESSAGE(WARNING "Couldn't found ARTEC SDK")
	else()
		list(APPEND RGBD_TOOLS_DEFINITIONS "HAS_ARTEC_SDK")
		MESSAGE(STATUS "Found ARTEC SDK, adding to current build")
	endif()
endif()

#########################################
# catking
if(${USE_CATKIN})
	find_package(catkin)
	if(${catkin_FOUND})
		catkin_package(
			CATKIN_DEPENDS roscpp
			INCLUDE_DIRS include
		)
		list(APPEND RGBD_TOOLS_DEFINITIONS "RGBDTOOLS_USE_ROS")
	endif(${catkin_FOUND})
endif(${USE_CATKIN})

#########################################
# Darknet - original version with CUDA compatibility 
if (${USE_DARKNET})
	add_subdirectory(ThirdParty/darknet)
	list(APPEND RGBD_TOOLS_DEFINITIONS "HAS_DARKNET")
	set(RGBD_TOOLS_COMPILE_OPTIONS "${RGBD_TOOLS_COMPILE_OPTIONS} -fPIC")
endif(${USE_DARKNET})

#########################################
# Darknet - OpenCL version from https://github.com/sowson/darknet 
if (${USE_DARKNET_CL})
	add_subdirectory(ThirdParty/darknet_cl)
	list(APPEND RGBD_TOOLS_DEFINITIONS "HAS_DARKNET_CL")
	set(RGBD_TOOLS_COMPILE_OPTIONS "${RGBD_TOOLS_COMPILE_OPTIONS} -fPIC")
endif(${USE_DARKNET_CL})

#########################################
######			Library 		   ######
#########################################
#add_definitions(-fopenmp)

# Collect all sources
file(GLOB_RECURSE RGDBTOOLS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE RGDBTOOLS_HEADER_FILES "include/*.h" "include/*.inl")
 
add_library(${PROJECT_NAME} SHARED ${RGDBTOOLS_HEADER_FILES} ${RGDBTOOLS_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

target_compile_definitions(${PROJECT_NAME} PUBLIC ${RGBD_TOOLS_DEFINITIONS})
string(REPLACE " " ";" RGBD_TOOLS_COMPILE_OPTIONS_REPLACED_FLAGS ${RGBD_TOOLS_COMPILE_OPTIONS})
target_compile_options(${PROJECT_NAME} PUBLIC ${RGBD_TOOLS_COMPILE_OPTIONS_REPLACED_FLAGS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${ARTEC_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ARTEC_LIBS})

if (${CUDA_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${CUDA_LIBRARIES})
endif (${CUDA_FOUND})

target_include_directories(${PROJECT_NAME} PUBLIC ${ZED_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ZED_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${QHULL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${QHULL_LIBRARIES})

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC lapack)

if(${USE_MODULE_SLAM})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${dlib_LIBRARIES})
	if(${USE_DBOW2})
		if((${DBoW2_FOUND} EQUAL "1") AND (${DLib_FOUND} EQUAL "1"))
			target_include_directories(${PROJECT_NAME} PUBLIC ${DBoW2_INCLUDE_DIR})
			target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DBoW2_LIBRARY})
			target_include_directories(${PROJECT_NAME} PUBLIC ${DLoopDetector_INCLUDE_DIR})
			target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DLoopDetector_LIBRARY})
			target_include_directories(${PROJECT_NAME} PUBLIC ${DLib_INCLUDE_DIR})
			target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DLib_LIBRARY})
		endif()
	endif(${USE_DBOW2})
	if(${g2o_FOUND})		
		target_include_directories(${PROJECT_NAME}  PUBLIC ${CHOLMOD_INCLUDE_DIR})
		target_link_libraries(${PROJECT_NAME}  LINK_PUBLIC g2o::core g2o::types_sba  g2o::solver_cholmod)
	endif()
endif(${USE_MODULE_SLAM})

if(${catkin_FOUND})
	target_include_directories(${PROJECT_NAME}  PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME}  LINK_PUBLIC  ${catkin_LIBRARIES})
	target_include_directories(${PROJECT_NAME}  PUBLIC ${rosbag_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME}  LINK_PUBLIC  ${rosbag_LIBRARIES})
endif(${catkin_FOUND})

if (${USE_DARKNET})
	target_include_directories(${PROJECT_NAME}  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/darknet/include")
	target_include_directories(${PROJECT_NAME}  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/darknet/src")
	target_link_libraries(${PROJECT_NAME}  LINK_PUBLIC darknet)
endif (${USE_DARKNET})

if (${USE_DARKNET_CL})
	target_include_directories(${PROJECT_NAME}  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/darknet_cl/darknet_cl/include")
	target_include_directories(${PROJECT_NAME}  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/darknet_cl/darknet_cl/src")
	target_link_libraries(${PROJECT_NAME}  LINK_PUBLIC darknet_cl)
endif (${USE_DARKNET_CL})


if(${LIBREALSENSE_FOUND})
    target_include_directories(${PROJECT_NAME} PUBLIC ${LIBREALSENSE_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${LIBREALSENSE_LIBRARIES})
endif()

if(${realsense2_FOUND})
    target_include_directories(${PROJECT_NAME} PUBLIC ${realsense2_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${realsense2_LIBRARY})
endif()

if(${LIBFREENECT_FOUND})
    target_include_directories(${PROJECT_NAME} PUBLIC ${LIBFREENECT_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${LIBFREENECT_LIBRARY})
endif()

#########################################
######			Documentation	   ######
#########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_rgbd_tools
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

#########################################
######		   	Tools 		   	   ######
#########################################
if(${BUILD_TOOLS})
	add_subdirectory(tools)
endif(${BUILD_TOOLS})

#########################################
######			Examples 		   ######
#########################################
if(${BUILD_EXAMPLES})
	add_subdirectory(examples)
endif(${BUILD_EXAMPLES})

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS rgbd_tools
        EXPORT rgbd_toolsTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
    rgbd_toolsConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )


# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

export(TARGETS rgbd_tools NAMESPACE rgbd_tools:: FILE rgbd_toolsTargets.cmake)

# This generates rgbd_toolsTargets.cmake
install(EXPORT rgbd_toolsTargets
    FILE rgbd_toolsTargets.cmake
    NAMESPACE rgbd_tools::
    DESTINATION lib/cmake/rgbd_tools
    )

configure_file(rgbd_toolsConfig.cmake.in rgbd_toolsConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rgbd_toolsConfig.cmake"
        DESTINATION lib/cmake/rgbd_tools
        )



# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
 
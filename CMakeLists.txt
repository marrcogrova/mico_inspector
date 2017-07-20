###############################################################
######                                                   ######
######  	RGBD TOOLS                                    ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(rgbdTools)

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(rgbd_tools)

#########################################
###### Loading 3rd party libraries.######
#########################################
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})
## Load librealsense
find_package(librealsense)
if(${LIBREALSENSE_FOUND})
	add_definitions(-DENABLE_LIBREALSENSE)
endif()

## Load opencv
find_package(OpenCV 3 REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

## Load PCL
find_package(PCL 1.8 QUIET REQUIRED)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

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
		add_definitions("-DHAS_ARTEC_SDK")
		MESSAGE(STATUS "Found ARTEC SDK, adding to current build")
	endif()
endif()


## Find CUDA and ZED SDKs
find_package(CUDA)
string(FIND ${CUDA_TOOLKIT_ROOT_DIR} NOTFOUND CUDA_FOUND)
if(${CUDA_FOUND} EQUAL -1)
	set(CUDA_FOUND true)
else()
	set(CUDA_FOUND false)
endif()

if(${CUDA_FOUND})
	if(WIN32)
		if (CMAKE_CL_64) # 64 bits	666 Check it properly...
			set(dummy $ENV{ZED_INCLUDE_DIRS})
			if(dummy MATCHES "")
				MESSAGE(WARNING "ZED environment variables are not defined")
			else()
				SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
				SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
				SET(ZED_LIBRARIES "${ZED_LIBRARY_DIR}/$ENV{ZED_LIBRARIES_64}")
				MESSAGE(STATUS "Found ZED 64 bits on Windows with CUDA 7.0")
			endif()
		else(CMAKE_CL_32) # 32 bits
			MESSAGE(WARNING "32 bits compilation is no more available with CUDA7.0")
		endif()
		
	elseif(UNIX)
		find_package(ZED)
	endif()
	
	if(NOT DEFINED ZED_LIBRARIES)
		MESSAGE(WARNING "Couldn't found or configure ZED SDK")
	else()
		add_definitions("-DHAS_ZED_SDK")
		MESSAGE(STATUS "Found ZED SDK, adding to current build")
	endif()
else()
	MESSAGE(WARNING "CUDA not found, ZED SDK cannot be configured")
	set(CUDA_INCLUDE_DIRS "")
	set(CUDA_LIBRARIES "")
endif()

#########################################
######			Library 		   ######
#########################################
set(PROJECT_NAME rgbd_slam)

# Collect all sources
file(GLOB_RECURSE RGDBTOOLS_SOURCE_FILES "rgbd_tools/*.cpp" "rgbd_tools/*.h" "rgbd_tools/*.inl")

add_library(${PROJECT_NAME} STATIC ${RGDBSLAM_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/rgbd_tools)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${ARTEC_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ARTEC_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${CUDA_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${ZED_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ZED_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${QHULL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${QHULL_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${LIBREALSENSE_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${LIBREALSENSE_LIBRARIES})

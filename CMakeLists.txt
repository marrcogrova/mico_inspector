###############################################################
######                                                   ######
######  	RGBD TOOLS                                    ######
######			Author: Pablo R.S. (aka. Bardo91)        ######
######                                                   ######
###############################################################

cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project(rgbd_tools)

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

set(USE_DARKNET ON)

set(RGBD_TOOLS_VERSION 0.2.2)
set(BUILD_TOOLS ON)
set(BUILD_EXAMPLES OFF)

#########################################
###### Loading 3rd party libraries.######
#########################################
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
elseif(APPLE)
    add_definitions(-Wno-narrowing -pthread)
    add_definitions(-std=gnu++11 -stdlib=libc++)
endif()
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})
## Load librealsense
find_package(librealsense)
if(${LIBREALSENSE_FOUND})
    MESSAGE(STATUS "Found librealsense v1, compiling RealSense devices wrapper")
    add_definitions(-DENABLE_LIBREALSENSE_V1)
else()
    find_package(realsense2 HINTS "/usr/local/lib/cmake/realsense2")

    if(${realsense2_FOUND})
        MESSAGE(STATUS "Found librealsense v2, compiling RealSense devices wrapper")
        add_definitions(-DENABLE_LIBREALSENSE_V2)
    endif()

endif()

## Load freenect
find_package(libfreenect)
if(${LIBFREENECT_FOUND})
    MESSAGE(STATUS "Found libfreenect, compiling kinect wrapper")
    add_definitions(-DENABLE_LIBFREENECT)
endif()

find_package(DBoW2)
find_package(DLib)

if((${DBoW2_FOUND} EQUAL "1") AND (${DLib_FOUND} EQUAL "1"))
	add_definitions(-DUSE_DBOW2)
endif()

## Load opencv
find_package(OpenCV 3 REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

## Load PCL
find_package(PCL 1.8.0 QUIET REQUIRED)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	add_definitions(${PCL_DEFINITIONS})
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

## Load PCL
find_package(dlib REQUIRED)

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

find_package(catkin)
if(${catkin_FOUND})
	catkin_package(
	  	CATKIN_DEPENDS roscpp
		INCLUDE_DIRS include
	)
endif(${catkin_FOUND})

# g2o dependencies
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(CSparse REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED)
FIND_PACKAGE(G2O REQUIRED)

if (${USE_DARKNET})
	find_package(CUDA)
		if (${CUDA_FOUND})
			find_package(Darknet)
			if (${Darknet_FOUND})
				add_definitions(-DHAS_DARKNET)
				add_definitions(-DGPU)
				link_directories(/usr/local/cuda/lib64)
				include_directories(/usr/local/cuda/include)
			else(${Darknet_FOUND})
				SET(HAS_DARKNET OFF)
			endif (${Darknet_FOUND})
		endif (${CUDA_FOUND})
endif(${USE_DARKNET})

#########################################
######			Library 		   ######
#########################################


# Collect all sources
GroupSources(include)
GroupSources(src)
file(GLOB_RECURSE RGDBTOOLS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE RGDBTOOLS_HEADER_FILES "include/*.h" "include/*.inl")

add_library(${PROJECT_NAME} STATIC ${RGDBTOOLS_HEADER_FILES} ${RGDBTOOLS_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

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

#target_include_directories(${PROJECT_NAME} PUBLIC ${dlib_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC dlib::dlib)

target_include_directories( ${PROJECT_NAME}	PUBLIC	${EIGEN_INCLUDE_DIRS}
													${CSPARSE_INCLUDE_DIR}
													${Cholmod_INCLUDE_DIR}
													${G2O_INCLUDE_DIR}
							)

LIST(APPEND G2O_LIBS
  cxsparse
  cholmod
  g2o_cli g2o_ext_freeglut_minimal g2o_simulator
  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
  g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
  g2o_types_sba g2o_types_slam3d g2o_csparse_extension
  g2o_opengl_helper g2o_solver_dense g2o_stuff
  g2o_types_sclam2d g2o_parser g2o_solver_pcg
  g2o_types_data g2o_types_sim3
)

target_link_libraries(${PROJECT_NAME} PUBLIC  ${G2O_LIBS})

if (${Darknet_FOUND})
	find_package(CUDA REQUIRED)
	target_include_directories(${PROJECT_NAME} PUBLIC ${Darknet_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Darknet_LIBRARIES} cuda cudart cublas curand cudnn)
endif (${Darknet_FOUND})


# OTHER OPTIONAL LIBRARIES
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

if((${DBoW2_FOUND} EQUAL "1") AND (${DLib_FOUND} EQUAL "1"))
    target_include_directories(${PROJECT_NAME} PUBLIC ${DBoW2_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DBoW2_LIBRARIES})

    target_include_directories(${PROJECT_NAME} PUBLIC ${DLib_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DLib_LIBRARY})
endif()



#########################################
######			Documentation	   ######
#########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc_rgbd_tools
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
if(${BUILD_EXAMPLEs})
	add_subdirectory(examples)
endif(${BUILD_EXAMPLEs})

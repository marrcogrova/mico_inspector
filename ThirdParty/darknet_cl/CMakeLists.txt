cmake_minimum_required(VERSION 3.7.2 FATAL_ERROR)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(darknet_cl LANGUAGES C CXX)

option(DARKNET_CPU "Enable cpu support" OFF)
option(DARKNET_OPENCV "Enable OpenCV support" ON)
option(DARKNET_OPENMP "Enable OpenMP support" OFF)
option(DARKNET_GPU "Enable OpenCL support" ON)
option(DARKNET_GPU_FAST "Enable OpenCL fast kernels support" ON)
option(DARKNET_GPU_MULTI "Enable OpenCL Multi-GPU support" OFF) # it is still experimental
option(DARKNET_RPI "Enable Raspberry PI support" OFF)
option(DARKNET_BENCHMARK "Some Benchmark Stats" OFF)
option(DARKNET_TESTING "Build the unit tests" OFF)
option(DARKNET_DEBUG "Enable debug mode" OFF)
option(DARKNET_STATIC "Build static library" OFF)
option(DARKNET_SHARED "Build shared library" ON)
option(DARKNET_NO_BINARY "Do not build binary" OFF)

include(GNUInstallDirs)

include_directories("include" "src")

set (DARKNET_DEFINITIONS "")
set (DARKNET_BINARY bindarknet)
set (DARKNET_LIBRARY libdarknet_cl)
set (DARKNET_LIBRARY_STATIC libdarknet_cl_s)

if (DARKNET_DEBUG OR DARKNET_TESTING)
    list (APPEND DARKNET_CFLAGS "-g")
else()
	if (DARKNET_OPENMP)
		list (APPEND DARKNET_CFLAGS "-fopenmp")
	endif()
	list (APPEND DARKNET_CFLAGS "-Wfatal-errors")
	list (APPEND DARKNET_CFLAGS "-Wno-unused-variable")
	list (APPEND DARKNET_CFLAGS "-O0")
endif()

# Find OpenCV
if (DARKNET_OPENCV)
	find_package(OpenCV REQUIRED)
	add_library(opencv INTERFACE IMPORTED)
	set_property(TARGET opencv PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})
    set_property(TARGET opencv PROPERTY
        INTERFACE_LINK_LIBRARIES ${OpenCV_LIBRARIES})
    list (APPEND DARKNET_DEFINITIONS OPENCV)
endif()

# Find OpenCL
if (DARKNET_GPU)
	find_package(OpenCL REQUIRED)
    add_library(opencl INTERFACE IMPORTED)
    set_property(TARGET opencl PROPERTY
    	INTERFACE_INCLUDE_DIRECTORIES ${OpenCL_INCLUDE_DIRS})
    set_property(TARGET opencl PROPERTY
        INTERFACE_LINK_LIBRARIES ${OpenCL_LIBRARY})
    list (APPEND DARKNET_DEFINITIONS GPU)
	# Raspberry PI
	if (DARKNET_RPI)
		list (APPEND DARKNET_DEFINITIONS RPI)
	else()
		find_package(clBLAS REQUIRED)
		add_library(clblas INTERFACE IMPORTED)
		set_property(TARGET clblas PROPERTY
			INTERFACE_INCLUDE_DIRECTORIES ${CLBLAS_INCLUDE_DIRS})
		set_property(TARGET clblas PROPERTY
			INTERFACE_LINK_LIBRARIES ${CLBLAS_LIBRARIES})
	endif()
endif()

# Fast Training
if (DARKNET_GPU_FAST)
	list (APPEND DARKNET_DEFINITIONS GPU_FAST)
endif()

# Multi-GPU Training
if (DARKNET_GPU_MULTI)
	list (APPEND DARKNET_DEFINITIONS GPU_MULTI)
endif()

# Benchmark Stats
if (DARKNET_BENCHMARK)
	list (APPEND DARKNET_DEFINITIONS BENCHMARK)
endif()

# Find Catch
if (DARKNET_UNIT)
	set (DARKNET_CPU OFF)
	find_package(Catch REQUIRED)
	add_library(catch INTERFACE IMPORTED)
	set_property(TARGET catch PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${Catch_INCLUDE_DIRS})
endif()

set (DARKNET_SOURCES
    "examples/art.c"
    "examples/attention.c"
    "examples/captcha.c"
    "examples/cifar.c"
    "examples/classifier.c"
    "examples/coco.c"
    "examples/detector.c"
    "examples/dice.c"
    "examples/go.c"
	"examples/instance-segmenter.c"
    "examples/lsd.c"
    "examples/nightmare.c"
    "examples/regressor.c"
    "examples/rnn.c"
    "examples/rnn_vid.c"
    "examples/segmenter.c"
    "examples/super.c"
    "examples/swag.c"
    "examples/tag.c"
    "examples/voxel.c"
    "examples/writing.c"
    "examples/yolo.c"
    "src/darknet_cl/activation_layer.c"
    "src/darknet_cl/activations.c"
    "src/darknet_cl/avgpool_layer.c"
    "src/darknet_cl/batchnorm_layer.c"
    "src/darknet_cl/blas.c"
    "src/darknet_cl/box.c"
    "src/darknet_cl/col2im.c"
    "src/darknet_cl/compare.c"
    "src/darknet_cl/connected_layer.c"
    "src/darknet_cl/convolutional_layer.c"
    "src/darknet_cl/cost_layer.c"
    "src/darknet_cl/crnn_layer.c"
    "src/darknet_cl/crop_layer.c"
    "src/darknet_cl/data.c"
    "src/darknet_cl/deconvolutional_layer.c"
    "src/darknet_cl/demo.c"
    "src/darknet_cl/detection_layer.c"
    "src/darknet_cl/dropout_layer.c"
    "src/darknet_cl/gemm.c"
    "src/darknet_cl/gru_layer.c"
    "src/darknet_cl/im2col.c"
    "src/darknet_cl/image.c"
	"src/darknet_cl/iseg_layer.c"
    "src/darknet_cl/layer.c"
    "src/darknet_cl/list.c"
    "src/darknet_cl/local_layer.c"
    "src/darknet_cl/lstm_layer.c"
    "src/darknet_cl/matrix.c"
    "src/darknet_cl/maxpool_layer.c"
    "src/darknet_cl/network.c"
    "src/darknet_cl/normalization_layer.c"
    "src/darknet_cl/option_list.c"
    "src/darknet_cl/parser.c"
    "src/darknet_cl/region_layer.c"
    "src/darknet_cl/reorg_layer.c"
    "src/darknet_cl/rnn_layer.c"
    "src/darknet_cl/route_layer.c"
    "src/darknet_cl/shortcut_layer.c"
    "src/darknet_cl/softmax_layer.c"
    "src/darknet_cl/tree.c"
    "src/darknet_cl/utils.c"
    "src/darknet_cl/yolo_layer.c"
    "src/darknet_cl/upsample_layer.c"
    "src/darknet_cl/logistic_layer.c"
    "src/darknet_cl/l2norm_layer.c"
)

set (DARKNET_PUBLIC_HEADERS
    "include/darknet_cl/darknet.h"
    "src/darknet_cl/activation_layer.h"
    "src/darknet_cl/activations.h"
    "src/darknet_cl/avgpool_layer.h"
    "src/darknet_cl/batchnorm_layer.h"
    "src/darknet_cl/blas.h"
    "src/darknet_cl/box.h"
    "src/darknet_cl/bugfixes.h"
    "src/darknet_cl/classifier.h"
    "src/darknet_cl/col2im.h"
    "src/darknet_cl/connected_layer.h"
    "src/darknet_cl/convolutional_layer.h"
    "src/darknet_cl/cost_layer.h"
    "src/darknet_cl/crnn_layer.h"
    "src/darknet_cl/crop_layer.h"
    "src/darknet_cl/opencl.h"
    "src/darknet_cl/data.h"
    "src/darknet_cl/deconvolutional_layer.h"
    "src/darknet_cl/demo.h"
    "src/darknet_cl/detection_layer.h"
    "src/darknet_cl/dropout_layer.h"
    "src/darknet_cl/gemm.h"
    "src/darknet_cl/gru_layer.h"
    "src/darknet_cl/im2col.h"
    "src/darknet_cl/image.h"
	"src/darknet_cl/iseg_layer.h"
    "src/darknet_cl/layer.h"
    "src/darknet_cl/list.h"
    "src/darknet_cl/local_layer.h"
	"src/darknet_cl/lstm_layer.h"
    "src/darknet_cl/matrix.h"
    "src/darknet_cl/maxpool_layer.h"
    "src/darknet_cl/network.h"
    "src/darknet_cl/normalization_layer.h"
    "src/darknet_cl/option_list.h"
    "src/darknet_cl/parser.h"
    "src/darknet_cl/region_layer.h"
    "src/darknet_cl/reorg_layer.h"
    "src/darknet_cl/rnn_layer.h"
    "src/darknet_cl/route_layer.h"
    "src/darknet_cl/shortcut_layer.h"
    "src/darknet_cl/softmax_layer.h"
    "src/darknet_cl/stb_image.h"
    "src/darknet_cl/stb_image_write.h"
    "src/darknet_cl/tree.h"
    "src/darknet_cl/unit.h"
    "src/darknet_cl/utils.h"
	"src/darknet_cl/opencl.h"
    "src/darknet_cl/yolo_layer.h"
    "src/darknet_cl/upsample_layer.h"
    "src/darknet_cl/logistic_layer.h"
    "src/darknet_cl/l2norm_layer.h"
)

set (DARKNET_MAIN_SOURCE
    "examples/darknet.c"
)

if (DARKNET_CPU)
	set (DARKNET_CPU_SOURCES
			"src/darknet_cl/cpu.c"
			)
endif()

if (DARKNET_GPU)
	set (DARKNET_GPU_SOURCES
	    "src/darknet_cl/activation_kernels.c"
	    "src/darknet_cl/activation_kernels.cl"
	    "src/darknet_cl/avgpool_layer_kernels.c"
	    "src/darknet_cl/avgpool_layer_kernels.cl"
	    "src/darknet_cl/blas_kernels.c"
	    "src/darknet_cl/blas_kernels.cl"
	    "src/darknet_cl/col2im_kernels.c"
	    "src/darknet_cl/col2im_kernels.cl"
	    "src/darknet_cl/convolutional_kernels.c"
	    "src/darknet_cl/convolutional_kernels.cl"
	    "src/darknet_cl/crop_layer_kernels.c"
	    "src/darknet_cl/crop_layer_kernels.cl"
	    "src/darknet_cl/deconvolutional_kernels.c"
	    "src/darknet_cl/dropout_layer_kernels.c"
	    "src/darknet_cl/dropout_layer_kernels.cl"
	    "src/darknet_cl/im2col_kernels.c"
	    "src/darknet_cl/im2col_kernels.cl"
	    "src/darknet_cl/maxpool_layer_kernels.c"
	    "src/darknet_cl/maxpool_layer_kernels.cl"
	    "src/darknet_cl/opencl.c"
	)
endif()

if (DARKNET_UNIT)
	set (DARKNET_UNIT_SOURCES
	    "src/darknet_cl/unit.c"
	    "src/darknet_cl/unit.h"
	    "src/darknet_cl/blas_unit.c"
	    "src/darknet_cl/col2im_unit.c"
	    "src/darknet_cl/convolutional_unit.c"
	    "src/darknet_cl/gemm_unit.c"
	    "src/darknet_cl/maxpool_unit.c"
	    "src/darknet_cl/network_unit.c"
	    "src/darknet_cl/region_unit.c"
	)
endif()

function (ConfigureBinary target)
    set_target_properties(${target} PROPERTIES OUTPUT_NAME "darknet_cl")    
    target_compile_definitions(${target} PUBLIC ${DARKNET_DEFINITIONS})
    target_compile_options(${target} PRIVATE ${DARKNET_CFLAGS})

    if (DARKNET_GPU)
        target_link_libraries(${target} opencl clblas)
    endif()

    if (DARKNET_OPENCV)
        target_link_libraries(${target} opencv)
    endif()

#    if (DARKNET_TESTING)
#        target_link_libraries(${target} catch)
#    endif()

    target_link_libraries(${target} "m" "pthread")
endfunction()

function (ConfigureLibrary target)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
    )
    install(TARGETS ${target} EXPORT ${CMAKE_PROJECT_NAME}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
endfunction()

if (DARKNET_SHARED)
    add_library(${DARKNET_LIBRARY} SHARED ${DARKNET_SOURCES} ${DARKNET_CPU_SOURCES}
	   ${DARKNET_GPU_SOURCES} ${DARKNET_WINDOWS_SOURCES})
    ConfigureBinary(${DARKNET_LIBRARY})
    ConfigureLibrary(${DARKNET_LIBRARY})
endif()

if (DARKNET_STATIC)
    add_library(${DARKNET_LIBRARY_STATIC} STATIC ${DARKNET_SOURCES} ${DARKNET_CPU_SOURCES}
       ${DARKNET_GPU_SOURCES} ${DARKNET_WINDOWS_SOURCES})
    ConfigureBinary(${DARKNET_LIBRARY_STATIC})
    ConfigureLibrary(${DARKNET_LIBRARY_STATIC})
endif()


if (NOT DARKNET_NO_BINARY)
    add_executable(${DARKNET_BINARY} ${DARKNET_SOURCES} ${DARKNET_CPU_SOURCES}
        ${DARKNET_GPU_SOURCES} ${DARKNET_WINDOWS_SOURCES}
        ${DARKNET_MAIN_SOURCE})

    ConfigureBinary(${DARKNET_BINARY})
endif()



install(  TARGETS darknet
          EXPORT rgbd_toolsTargets
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )


# install darknet files too
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY src/darknet_cl DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h")
	


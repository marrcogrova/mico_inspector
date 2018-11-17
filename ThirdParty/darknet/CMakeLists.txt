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
project(darknet VERSION 0.3)

##########################################
######		Configure project		######
##########################################

#########################################
###### Loading 3rd party libraries.######
#########################################

#########################################
######			Library 		   ######
#########################################
#add_definitions(-fopenmp)

# Collect all sources
file(GLOB_RECURSE DARKNET_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE DARKNET_HEADER_FILES "include/*.h" "include/*.inl")
 
add_library(${PROJECT_NAME} static ${DARKNET_HEADER_FILES} ${DARKNET_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

target_compile_definitions(${PROJECT_NAME} PUBLIC )
target_compile_options(${PROJECT_NAME} PUBLIC )
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

file(GLOB_RECURSE DARKNET_EXAMPLE_SOURCE_FILES "src/*.cpp" "src/*.c")
add_executable(${PROJECT_NAME}_application ${DARKNET_EXAMPLE_SOURCE_FILES})



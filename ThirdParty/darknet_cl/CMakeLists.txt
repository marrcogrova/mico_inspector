cmake_minimum_required(VERSION 2.8)

project(darknet_cl LANGUAGES C CXX)


find_package(OpenCL)
find_package(OpenCV)


file(GLOB_RECURSE DARKNET_CL_HEADER_FILES "darknet_cl/include/*.h" "darknet_cl/include/*.inl")
file(GLOB_RECURSE DARKNET_CL_SRC_FILES "darknet_cl/src/*.c" "darknet_cl/src/*.cpp")
file(GLOB_RECURSE DARKNET_CL_EXAMPLE_FILES "darknet_cl/example/*.c" "darknet_cl/example/*.cpp")

add_library(${PROJECT_NAME} SHARED ${DARKNET_CL_HEADER_FILES} ${DARKNET_CL_SRC_FILES})
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DGPU=1")
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_include_directories(${PROJECT_NAME} PUBLIC "darknet_cl/include")
target_include_directories(${PROJECT_NAME} PUBLIC "darknet_cl/clBLAS") 

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCL_LIBRARY})
target_link_libraries(${PROJECT_NAME} LINK_PRIVATE ${PROJECT_SOURCE_DIR}/darknet_cl/clBLAS/libclBLAS.so )
target_link_libraries(${PROJECT_NAME} LINK_PRIVATE pthread)

#-----------------------------------------
add_executable(${PROJECT_NAME}_app ${DARKNET_CL_HEADER_FILES} ${DARKNET_CL_SRC_FILES} ${DARKNET_CL_EXAMPLE_FILES})
target_compile_definitions(${PROJECT_NAME}_app PUBLIC "-DGPU=1")
target_compile_features(${PROJECT_NAME}_app PUBLIC cxx_std_11)

target_include_directories(${PROJECT_NAME}_app PUBLIC "darknet_cl/include")
target_include_directories(${PROJECT_NAME}_app PUBLIC "darknet_cl/clBLAS") 

target_include_directories(${PROJECT_NAME}_app PUBLIC ${OpenCL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_app LINK_PUBLIC ${OpenCL_LIBRARY})
target_link_libraries(${PROJECT_NAME}_app LINK_PRIVATE ${PROJECT_SOURCE_DIR}/darknet_cl/clBLAS/libclBLAS.so )
target_link_libraries(${PROJECT_NAME}_app LINK_PRIVATE pthread)


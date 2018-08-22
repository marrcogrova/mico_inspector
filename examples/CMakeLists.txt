macro(build_example example_name example_files)
    add_executable(${example_name} ${example_files})
    target_include_directories(${example_name} PUBLIC ../../include)
    target_link_libraries(${example_name} LINK_PUBLIC rgbd_tools)

	if (${USE_DARKNET})
	    target_link_libraries(${example_name} LINK_PUBLIC darknet)
	endif()
endmacro()

build_example(color_clustering color_clustering/main.cpp)
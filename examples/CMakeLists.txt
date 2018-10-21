macro(build_example example_name example_files)
    add_executable(${example_name} ${${example_files}})
    target_include_directories(${example_name} PUBLIC ../include)
    target_link_libraries(${example_name} LINK_PUBLIC rgbd_tools)

	if (${USE_DARKNET})
	    target_link_libraries(${example_name} LINK_PUBLIC darknet)
	endif()
endmacro()

set(COLOR_CLUSTERING_EXAMPLE_FILES color_clustering/main.cpp)
build_example(example_color_clustering COLOR_CLUSTERING_EXAMPLE_FILES)

set(EKF_EXAMPLE_FILES state_filtering/ekf/main.cpp)
build_example(example_ekf EKF_EXAMPLE_FILES)

set(BA_EXAMPLE_FILES ba/main.cpp)
build_example(example_ba BA_EXAMPLE_FILES)

set(PARTICLE_FILTER_EXAMPLE_FILES   state_filtering/particle_filter/main.cpp 
                                    state_filtering/particle_filter/Robot.h 
                                    state_filtering/particle_filter/Robot.cpp)
build_example(example_particle_filter PARTICLE_FILTER_EXAMPLE_FILES)

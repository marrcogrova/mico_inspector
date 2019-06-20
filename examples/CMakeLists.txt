macro(build_example example_name example_files)
    add_executable(${example_name} ${${example_files}})
    target_include_directories(${example_name} PUBLIC ../include)
    target_link_libraries(${example_name} LINK_PUBLIC rgbd_tools )

    find_package( Boost COMPONENTS program_options )
    target_link_libraries( ${example_name} LINK_PUBLIC ${Boost_LIBRARIES} )

	if (${USE_DARKNET})
	    target_link_libraries(${example_name} LINK_PUBLIC darknet)
	endif()
endmacro()

#set(COLOR_CLUSTERING_EXAMPLE_FILES color_clustering/main.cpp)
#build_example(example_color_clustering COLOR_CLUSTERING_EXAMPLE_FILES)

#set(EKF_EXAMPLE_FILES state_filtering/ekf/main.cpp)
#build_example(example_ekf EKF_EXAMPLE_FILES)

if(${USE_MODULE_SLAM})
    set(BA_EXAMPLE_FILES ba/main.cpp)
    build_example(example_ba BA_EXAMPLE_FILES)
endif(${USE_MODULE_SLAM})

#set(PARTICLE_FILTER_EXAMPLE_FILES   state_filtering/particle_filter/example_2d.cpp )
#build_example(example_particle_filter_2d PARTICLE_FILTER_EXAMPLE_FILES)

#set(PARTICLE_FILTER_EXAMPLE_FILES   state_filtering/particle_filter/example_3d.cpp )
#build_example(example_particle_filter_3d PARTICLE_FILTER_EXAMPLE_FILES)

#set(ML_LDA_EXAMPLE_FILES   ml/classification/example_topic_modeling.cpp )
#build_example(example_ml_lda ML_LDA_EXAMPLE_FILES)

set(HAL_EXAMPLE_FILES   hal/imu-pololu/minimu9-ahrs.cpp)
build_example(example-minimu HAL_EXAMPLE_FILES)

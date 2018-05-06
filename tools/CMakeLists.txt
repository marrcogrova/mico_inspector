macro(build_tool tool_name tool_files)
    add_executable(${tool_name} ${tool_files})
    target_include_directories(${tool_name} PUBLIC ../../include)
    target_link_libraries(${tool_name} LINK_PUBLIC rgbd_tools)
    target_link_libraries(${tool_name} LINK_PUBLIC darknet)
endmacro()

build_tool(dataset_recorder dataset_recorder/main.cpp)
build_tool(feature_object_tracking feature_object_tracking/main.cpp)
build_tool(yolo_detector yolo_detector/main.cpp)


find_package(catkin COMPONENTS cv_bridge image_transport)
if(${catkin_FOUND})

	catkin_package(
	  	CATKIN_DEPENDS roscpp
	)

	macro(build_ros_tool tool_name tool_files)
	    add_executable(${tool_name} ${tool_files})
	    target_include_directories(${tool_name} PUBLIC ../../include)
	    target_link_libraries(${tool_name} LINK_PUBLIC rgbd_tools)
	    target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
	    target_link_libraries(${tool_name} ${catkin_LIBRARIES})
	endmacro()

	build_ros_tool(ros_publisher ros_publisher/main.cpp)

endif(${catkin_FOUND})


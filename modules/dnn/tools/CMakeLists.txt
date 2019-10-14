macro(build_tool tool_name tool_files)
    add_executable(${tool_name} ${tool_files})
    target_include_directories(${tool_name} PRIVATE ../include)
    target_link_libraries(${tool_name} LINK_PRIVATE mico::mico-base)
    target_link_libraries(${tool_name} LINK_PRIVATE mico::mico-dnn)
	
	if (${USE_DARKNET})	
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet/include")	
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet/src")
	endif(${USE_DARKNET})
	if (${USE_DARKNET_CL})
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet_cl/darknet_cl/include")
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet_cl/darknet_cl/src")
	endif(${USE_DARKNET_CL})
endmacro()


if (${USE_DARKNET})
 	build_tool(yolo_detector yolo_detector/main.cpp)
endif (${USE_DARKNET})
if (${USE_DARKNET_CL})
	build_tool(yolo_detector_cl yolo_detector_cl/main.cpp)
endif (${USE_DARKNET_CL})


# find_package(catkin COMPONENTS cv_bridge image_transport)
# if(${catkin_FOUND})

# 	catkin_package(
# 	  	CATKIN_DEPENDS roscpp
# 	)

# 	macro(build_ros_tool tool_name tool_files)
# 	    add_executable(${tool_name} ${tool_files})
#  	    target_include_directories(${tool_name} PUBLIC ../include)
# 	    target_link_libraries(${tool_name} LINK_PUBLIC rgbd_tools)
# 	    target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
# 	    target_link_libraries(${tool_name} ${catkin_LIBRARIES})
# 	endmacro()

# 	# build_ros_tool(ros_publisher ros_publisher/main.cpp)

# endif(${catkin_FOUND})


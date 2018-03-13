macro(build_tool tool_name tool_files)
    add_executable(${tool_name} ${tool_files})
    target_include_directories(${tool_name} PUBLIC ${RGDBTOOLS_HEADER_FILES})
    target_link_libraries(${tool_name} LINK_PUBLIC rgbd_tools)
endmacro()

build_tool(dataset_recorder dataset_recorder/main.cpp)
add_executable(RobotReplayer 
main.cpp 
)

target_compile_definitions(RobotReplayer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(RobotReplayer PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
utils
robotutils
)

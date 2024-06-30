add_executable(RippleVisualization 
main.cpp
)

target_compile_definitions(RippleVisualization PRIVATE vsgXchange_FOUND)
target_compile_definitions(RippleVisualization PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(RippleVisualization PRIVATE NOMINMAX=1)
endif(WIN32)


target_link_libraries(RippleVisualization 
PUBLIC 
Eigen3::Eigen
robotutils
utils
renderable
${OPERATING_SYSTEM_REQUIRMENTS}
)
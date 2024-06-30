add_executable(LimitCycleController 
main.cpp
)

target_compile_definitions(LimitCycleController PRIVATE vsgXchange_FOUND)
target_compile_definitions(LimitCycleController PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(LimitCycleController PRIVATE NOMINMAX=1)
endif(WIN32)


target_link_libraries(LimitCycleController 
PUBLIC 
utils
robotutils
renderable
${OPERATING_SYSTEM_REQUIRMENTS}
)
find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(RealTimeReconstructor
main.cpp
)

target_compile_definitions(RealTimeReconstructor PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeReconstructor PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(RealTimeReconstructor PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(RealTimeReconstructor
PUBLIC
${ITK_LIBRARIES}
communication
imageprocessing
vsg::vsg
vsgXchange::vsgXchange
renderable
utils
)

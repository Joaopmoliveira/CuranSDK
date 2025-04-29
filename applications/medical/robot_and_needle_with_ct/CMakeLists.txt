find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(RealTimeNavigation3D
main.cpp
)

target_compile_definitions(RealTimeNavigation3D PRIVATE vsgXchange_FOUND)
target_compile_definitions(RealTimeNavigation3D PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(RealTimeNavigation3D PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(RealTimeNavigation3D
PUBLIC
${ITK_LIBRARIES}
communication
imageprocessing
vsg::vsg
vsgXchange::vsgXchange
renderable
utils
robotutils
)

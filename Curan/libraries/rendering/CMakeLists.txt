find_package(vsg )
find_package(vsgXchange )

add_library(renderable STATIC
src/Box.cpp
src/Capsule.cpp
src/Cylinder.cpp
src/DynamicTexture.cpp
src/Renderable.cpp
src/Sphere.cpp
src/Window.cpp
src/SequencialLinks.cpp
src/Floor.cpp
src/PhaseWiredBox.cpp
src/Volume.cpp
)

target_include_directories(renderable
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(renderable PUBLIC
vsg::vsg
vsgXchange::vsgXchange
utils
imageprocessing
)

target_compile_definitions(renderable PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
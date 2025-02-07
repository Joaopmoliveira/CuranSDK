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
src/DynamicHeight.cpp
src/Volume.cpp
src/ImGUIInterface.cpp
src/Arrow.cpp
src/Mesh.cpp
)

target_include_directories(renderable
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(renderable PUBLIC
utils
vsg::vsg
vsgXchange::vsgXchange
vsgImGui::vsgImGui
)

target_compile_definitions(renderable PUBLIC NOMINMAX)
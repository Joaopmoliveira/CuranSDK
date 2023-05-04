find_package(vsg REQUIRED)
find_package(vsgXchange REQUIRED)
find_package(Assimp REQUIRED)

add_library(renderable STATIC
src/Box.cpp
src/Capsule.cpp
src/Cylinder.cpp
src/DynamicTexture.cpp
src/Renderable.cpp
src/Sphere.cpp
src/Window.cpp
)

target_include_directories(renderable
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(renderable PUBLIC
vsg::vsg
vsgXchange::vsgXchange
utils
)
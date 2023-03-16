find_package(Vulkan REQUIRED)

add_library(userinterface STATIC
src/Context.cpp
src/Signal.cpp
src/Window.cpp
src/Button.cpp
src/SingletonIconResources.cpp 
src/DrawableImageBox.cpp)

target_include_directories(userinterface
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_include_directories(userinterface PUBLIC $<TARGET_PROPERTY:skia,INTERFACE>)

target_link_libraries(userinterface PUBLIC
asio
utils
OpenIGTLink
skia
glfw 
Vulkan::Vulkan 
)
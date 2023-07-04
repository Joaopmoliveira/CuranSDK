find_package(Vulkan REQUIRED)

add_library(userinterfacemodified STATIC
src/Context.cpp
src/Signal.cpp
src/Window.cpp
src/Button.cpp
src/IconResources.cpp
src/Container.cpp 
src/Drawable.cpp
src/Page.cpp
src/LightWeightPage.cpp 
src/Overlay.cpp
src/Slider.cpp
src/TextBlob.cpp
src/OpenIGTLinkViewer.cpp
)

target_include_directories(userinterfacemodified
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(userinterfacemodified PUBLIC
asio
utils
OpenIGTLink
unofficial::skia::skia
glfw 
Vulkan::Vulkan 
)


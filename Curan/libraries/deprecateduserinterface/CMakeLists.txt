find_package(Vulkan REQUIRED)

add_library(userinterface STATIC
src/Context.cpp
src/Signal.cpp
src/Window.cpp
src/Button.cpp
src/IconResources.cpp
src/OpenIGTLinkViewer.cpp
src/Container.cpp 
src/Drawable.cpp
src/Page.cpp
src/ImageDisplay.cpp
src/RadioButton.cpp
src/Slider.cpp 
src/TextBlob.cpp 
src/LightWeightPage.cpp 
src/Overlay.cpp)

target_include_directories(userinterface
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(userinterface PUBLIC
asio
utils
OpenIGTLink
unofficial::skia::skia
glfw 
Vulkan::Vulkan 
)


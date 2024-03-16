find_package(Vulkan REQUIRED)

add_subdirectory(modules)

add_library(userinterface STATIC
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
src/ImageDisplay.cpp
src/ImageWrapper.cpp
src/Interactive.cpp
src/Panel.cpp
src/MiniPage.cpp
src/SliderPanel.cpp
src/MutatingTextPanel.cpp
src/ImutableTextPanel.cpp
src/ItemExplorer.cpp
src/Loader.cpp
)

find_package(ITK COMPONENTS ITKCommon ITKIOImageBase ITKIOGDCM ITKIOJPEG ITKIOTIFF ITKImageIO REQUIRED)

include(${ITK_USE_FILE})


target_include_directories(userinterface
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(userinterface PUBLIC
${ITK_LIBRARIES}
asio
utils
OpenIGTLink
unofficial::skia::skia
glfw 
Vulkan::Vulkan 
skplaintexteditor
)


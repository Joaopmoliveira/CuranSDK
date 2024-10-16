add_library(skplaintexteditor STATIC
src/editor.cpp
src/shape.cpp
src/stringslice.cpp
src/word_boundaries.cpp
)

target_include_directories(skplaintexteditor
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(skplaintexteditor PUBLIC
unofficial::skia::skia
unofficial::skia::modules::skunicode_icu
glfw 
Vulkan::Vulkan 
skshaper
)
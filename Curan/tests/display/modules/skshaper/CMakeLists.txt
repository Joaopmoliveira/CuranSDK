add_library(skshaper STATIC
src/SkShaper.cpp
#src/SkShaper_coretext.cpp
src/SkShaper_primitive.cpp
#src/SkShaper_harfbuzz.cpp
#src/SkShaper_skunicode.cpp
)

target_include_directories(skshaper
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(skshaper PUBLIC
unofficial::skia::skia
)
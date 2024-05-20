add_library(skunicode STATIC
#src/SkUnicode_client.cpp
#src/SkUnicode_hardcoded.cpp
#src/SkUnicode_icu_bidi.cpp
#src/SkUnicode_icu_builtin.cpp
#src/SkUnicode_icu_runtime.cpp
#src/SkUnicode_icu.cpp
#src/SkUnicode_icu4x.cpp
#src/SkUnicode_libgrapheme.cpp
src/SkUnicode.cpp
)

target_include_directories(skunicode
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(skunicode PUBLIC
unofficial::skia::skia
)
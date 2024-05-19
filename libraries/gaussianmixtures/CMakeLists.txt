add_library(gaussianmixture INTERFACE
)

target_include_directories(gaussianmixture
INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(gaussianmixture INTERFACE
utils
)
find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(isofurface_extraction isofurface_extraction.cpp )

target_compile_definitions(isofurface_extraction PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(isofurface_extraction PUBLIC
imageprocessing
${ITK_LIBRARIES}
)

add_executable(isofurface_extraction_arun_strategy isofurface_extraction_arun_strategy.cpp )

target_compile_definitions(isofurface_extraction_arun_strategy PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(isofurface_extraction_arun_strategy PUBLIC
imageprocessing
${ITK_LIBRARIES}
)
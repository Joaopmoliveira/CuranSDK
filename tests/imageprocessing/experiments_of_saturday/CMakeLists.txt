find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(isofurface_extraction isofurface_extraction.cpp )

target_link_libraries(isofurface_extraction PUBLIC
imageprocessing
${ITK_LIBRARIES}
)
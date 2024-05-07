find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(test_subvolume_generator subvolume_generator.cpp)

target_link_libraries(test_subvolume_generator
PUBLIC
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_volumes register_volumes.cpp)

target_link_libraries(test_register_volumes
PUBLIC
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)


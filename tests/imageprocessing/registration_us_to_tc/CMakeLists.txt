find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(test_subvolume_generator subvolume_generator.cpp)

target_compile_definitions(test_subvolume_generator PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_subvolume_generator
PUBLIC
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_volumes register_volumes.cpp)

target_compile_definitions(test_register_volumes PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_volumes
PUBLIC
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)


add_executable(test_register_volumes_v4 register_volumes_v4.cpp)

target_compile_definitions(test_register_volumes_v4 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_volumes_v4
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_volumes_v5 register_volumes_v5.cpp)

target_compile_definitions(test_register_volumes_v5 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_volumes_v5
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_resample resample.cpp)

target_compile_definitions(test_resample PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_resample
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

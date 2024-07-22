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


add_executable(test_apply_laplacian apply_laplacian.cpp)

target_compile_definitions(test_apply_laplacian PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_apply_laplacian
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_us_preprocessing us_preprocess.cpp)

target_compile_definitions(test_us_preprocessing PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_us_preprocessing
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_with_pca_and_multithreaded test_with_pca_and_multithreaded.cpp)

target_compile_definitions(test_with_pca_and_multithreaded PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_with_pca_and_multithreaded
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_volumes_v6 register_volumes_v6.cpp)

target_compile_definitions(test_register_volumes_v6 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_volumes_v6
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_volumes_v7 register_volumes_v7.cpp)

target_compile_definitions(test_register_volumes_v7 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_volumes_v7
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_apply_laplacianv2 apply_laplacianv2.cpp)

target_compile_definitions(test_apply_laplacianv2 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_apply_laplacianv2
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_volumes_v8 register_volumes_v8.cpp)

target_compile_definitions(test_register_volumes_v8 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_volumes_v8
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_pca register_pca_aproach.cpp)

target_compile_definitions(test_register_pca PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_pca
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_register_volumes_v9 register_volumes_v9.cpp)

target_compile_definitions(test_register_volumes_v9 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_register_volumes_v9
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_apply_laplacian_v3 apply_laplacianv3.cpp)

target_compile_definitions(test_apply_laplacian_v3 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_apply_laplacian_v3
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_final_icp_registration final_icp_registration.cpp)

target_compile_definitions(test_final_icp_registration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_final_icp_registration
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)

add_executable(test_icp_mi_registration icp_mi_registration.cpp)

target_compile_definitions(test_icp_mi_registration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_icp_mi_registration
PUBLIC
utils
${ITK_LIBRARIES}
${OPERATING_SYSTEM_REQUIRMENTS}
)
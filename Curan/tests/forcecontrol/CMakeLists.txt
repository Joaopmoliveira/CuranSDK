add_executable(test_force_control main.cpp)

target_compile_definitions(test_force_control PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_force_control PUBLIC
utils
gaussianmixture
vsgps
)



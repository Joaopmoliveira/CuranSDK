add_executable(cpp_generic_api_testing main.cpp)

target_link_libraries(cpp_generic_api_testing PUBLIC 
utils
)

add_executable(test_variant_callback_implementation test_variant_storage_api.cpp)

add_executable(test_new_capture_buffer_arquitecture test_new_capture_buffer_arquitecture.cpp)

target_link_libraries(test_new_capture_buffer_arquitecture PUBLIC 
utils
)

add_executable(test_std_cin_limitations test_std_cin_limitations.cpp)

add_executable(test_cpp_laucher test_cpp_laucher.cpp)

IF (WIN32)
  add_compile_definitions(test_cpp_laucher CURAN_WINDOWS)
ELSEIF(UNIX)
  add_compile_definitions(test_cpp_laucher CURAN_LINUX)
ENDIF()

add_executable(test_dummy test_secondary_dummy_project.cpp)

add_executable(test_chrono_duration_propagation test_chrono_duration_propagation.cpp)
add_executable(test_util test_utils.cpp)

target_link_libraries(test_util PUBLIC
utils
)

add_executable(testing_cpp_mechanics testing_cpp_mechanics.cpp)
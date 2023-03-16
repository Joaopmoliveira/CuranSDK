add_executable(test_util test_utils.cpp)

target_link_libraries(test_util PUBLIC
utils
Pal::Sigslot
)
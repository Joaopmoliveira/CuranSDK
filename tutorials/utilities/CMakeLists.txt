add_executable(tutorial_safe_queue tutorial_safe_queue.cpp)

target_link_libraries(tutorial_safe_queue PUBLIC
utils
)

add_executable(tutorial_flag tutorial_flag.cpp)

target_link_libraries(tutorial_flag PUBLIC
utils
)
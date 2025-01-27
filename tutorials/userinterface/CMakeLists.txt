find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})


add_executable(full_tutorial_suite_ui full_tutorial_suite.cpp)
target_compile_definitions(full_tutorial_suite_ui PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(full_tutorial_suite_ui PUBLIC
utils
userinterface
${ITK_LIBRARIES}
)
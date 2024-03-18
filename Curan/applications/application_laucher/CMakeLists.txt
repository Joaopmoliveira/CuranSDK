add_executable(ApplicationLauncher
main.cpp
)

target_compile_definitions(ApplicationLauncher PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")


target_link_libraries(ApplicationLauncher PUBLIC
userinterface
)

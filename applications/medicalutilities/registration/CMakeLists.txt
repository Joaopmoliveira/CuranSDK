find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})


add_executable(RegistrationApplication
main.cpp
)

if(WAS_PLUS_SPECIFIED)
    target_compile_definitions(RegistrationApplication PRIVATE
    CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}"
    CURAN_PLUS_EXECUTABLE_PATH="${CURAN_PLUS_EXECUTABLE_PATH}")
else()
    target_compile_definitions(RegistrationApplication PRIVATE
    CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
endif()

target_link_libraries(RegistrationApplication PUBLIC
userinterface
${ITK_LIBRARIES}
${Boost_LIBRARIES}
)


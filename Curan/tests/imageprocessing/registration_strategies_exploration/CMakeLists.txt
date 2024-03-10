find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(explore_registration_mi_mattes
main_mi_mattes.cpp
)

target_compile_definitions(explore_registration_mi_mattes PRIVATE vsgXchange_FOUND)
target_compile_definitions(explore_registration_mi_mattes PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(explore_registration_mi_mattes
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
renderable
${OPERATING_SYSTEM_REQUIRMENTS}
)

#add_executable(explore_registration_v4_mi_mattes
#main.cpp
#)

#target_compile_definitions(explore_registration_v4_mi_mattes PRIVATE vsgXchange_FOUND)
#target_compile_definitions(explore_registration_v4_mi_mattes PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
#target_link_libraries(explore_registration_v4_mi_mattes
#PUBLIC
#${ITK_LIBRARIES}
#vsg::vsg
#vsgXchange::vsgXchange
#renderable
#${OPERATING_SYSTEM_REQUIRMENTS}
#)

#add_executable(explore_registration_mi_normalized
#main.cpp
#)

#target_compile_definitions(explore_registration_mi_normalized PRIVATE vsgXchange_FOUND)
#target_compile_definitions(explore_registration_mi_normalized PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
#target_link_libraries(explore_registration_mi_normalized
#PUBLIC
#${ITK_LIBRARIES}
#vsg::vsg
#vsgXchange::vsgXchange
#renderable
#${OPERATING_SYSTEM_REQUIRMENTS}
#)
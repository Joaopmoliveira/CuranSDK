find_package(vsg)
find_package(vsgXchange)

add_executable(Renderer
main.cpp
)

target_compile_definitions(Renderer PRIVATE vsgXchange_FOUND)
target_compile_definitions(Renderer PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(Renderer
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)


add_executable(VolumeAttempt
volume.cpp
)

target_compile_definitions(VolumeAttempt PRIVATE vsgXchange_FOUND)
target_compile_definitions(VolumeAttempt PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(VolumeAttempt
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)

add_executable(UpdateSceneAttempt
test_updatable_image.cpp
)

target_compile_definitions(UpdateSceneAttempt PRIVATE vsgXchange_FOUND)
target_compile_definitions(UpdateSceneAttempt PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(UpdateSceneAttempt
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)

add_executable(VolumeAttempt2
volume2.cpp
)

target_compile_definitions(VolumeAttempt2 PRIVATE vsgXchange_FOUND)
target_compile_definitions(VolumeAttempt2 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(VolumeAttempt2
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)



add_executable(VolumeAttempt3
volume3.cpp
)

target_compile_definitions(VolumeAttempt3 PRIVATE vsgXchange_FOUND)
target_compile_definitions(VolumeAttempt3 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(VolumeAttempt3
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)

find_package(ITK CONFIG REQUIRED)

include(${ITK_USE_FILE})

add_executable(ItkIntegration
test_itk_merge_vsg.cpp
)

target_compile_definitions(ItkIntegration PRIVATE vsgXchange_FOUND)
target_compile_definitions(ItkIntegration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(ItkIntegration
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
)

add_executable(VolumeAttempt4
volume4.cpp
)

target_compile_definitions(VolumeAttempt4 PRIVATE vsgXchange_FOUND)
target_compile_definitions(VolumeAttempt4 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(VolumeAttempt4
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
)

add_executable(VolumeAttempt5
volume5_transfer.cpp
)

target_compile_definitions(VolumeAttempt5 PRIVATE vsgXchange_FOUND)
target_compile_definitions(VolumeAttempt5 PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(VolumeAttempt5
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
)

add_executable(RobotDisplay
test_robot_display.cpp
)

target_compile_definitions(RobotDisplay PRIVATE vsgXchange_FOUND)
target_compile_definitions(RobotDisplay PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(RobotDisplay
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)
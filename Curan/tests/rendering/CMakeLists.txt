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
renderable
)

add_executable(test_append_functionality
test_append_functionality.cpp
)

target_compile_definitions(test_append_functionality PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_append_functionality PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_append_functionality
PUBLIC
renderable
)

add_executable(test_append_dynamic_ultrasound
test_append_with_dynamic_image_update.cpp
)

target_compile_definitions(test_append_dynamic_ultrasound PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_append_dynamic_ultrasound PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_append_dynamic_ultrasound
PUBLIC
renderable
)

add_executable(test_robot_with_ultrasound
test_robot_with_ultrasound.cpp
)

target_compile_definitions(test_robot_with_ultrasound PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_robot_with_ultrasound PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_robot_with_ultrasound
PUBLIC
renderable
)

add_executable(test_creation_of_wired_box
test_creation_of_wired_box.cpp
)

target_compile_definitions(test_creation_of_wired_box PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_creation_of_wired_box PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_creation_of_wired_box
PUBLIC
renderable
)

add_executable(test_world_position
validate_world_position_of_rendered_model.cpp
)

target_compile_definitions(test_world_position PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_world_position PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

if(WIN32) #FRILibrary pulls in macros which cause vsg to fail its compilation step thus we must define this macro when compiling for windows
    target_compile_definitions(test_world_position PRIVATE NOMINMAX=1)
endif(WIN32)

target_link_libraries(test_world_position
PUBLIC
KukaRobotLibrary 
FRILibrary
robotParameters
vsg::vsg
vsgXchange::vsgXchange
renderable
)
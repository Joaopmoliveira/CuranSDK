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

add_executable(test_volume_api
test_volume_api.cpp
)

target_compile_definitions(test_volume_api PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_volume_api PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_volume_api
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
renderable
)

add_executable(test_volume_api_update
test_volume_api_update.cpp
)

target_compile_definitions(test_volume_api_update PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_volume_api_update PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_volume_api_update
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
renderable
)

add_executable(test_visualization_registration
test_visualization_registration.cpp
)

target_compile_definitions(test_visualization_registration PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_visualization_registration PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(test_visualization_registration
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
renderable
userinterface
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

add_executable(test_world_position
validate_world_position_of_rendered_model.cpp
)

target_compile_definitions(test_world_position PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_world_position PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_world_position
PUBLIC
KukaRobotLibrary 
robotParameters
vsg::vsg
vsgXchange::vsgXchange
renderable
)

add_executable(test_empty_window
test_window_api.cpp
)

target_compile_definitions(test_empty_window PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_empty_window PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_empty_window
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
)

add_executable(test_heightfield 
test_heightfield.cpp
)

target_compile_definitions(test_heightfield PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_heightfield PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_heightfield
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
)

add_executable(test_user_interface_3d
test_ui_rendering.cpp
)

target_compile_definitions(test_user_interface_3d PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_user_interface_3d PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_user_interface_3d
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
vsgImGui::vsgImGui
)

add_executable(test_robot_with_ui
test_robot_with_ui.cpp
)

target_compile_definitions(test_robot_with_ui PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_robot_with_ui PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_robot_with_ui
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
vsgImGui::vsgImGui
)

add_executable(test_scenegraph_deletion
test_deletion_from_scene_graph.cpp
)

target_compile_definitions(test_scenegraph_deletion PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_scenegraph_deletion PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_scenegraph_deletion
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
vsgImGui::vsgImGui
)


add_executable(test_arrow
test_arrow.cpp
)

target_compile_definitions(test_arrow PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_arrow PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_arrow
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
vsgImGui::vsgImGui
)

add_executable(test_robot_and_imgui_plot
test_robot_visualization_with_joint_angles_real_time.cpp
)

target_compile_definitions(test_robot_and_imgui_plot PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_robot_and_imgui_plot PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_robot_and_imgui_plot
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
vsgImGui::vsgImGui
)


add_executable(test_swing_car
swing_car_display.cpp
)

target_compile_definitions(test_swing_car PRIVATE vsgXchange_FOUND)
target_compile_definitions(test_swing_car PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_swing_car
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
vsgImGui::vsgImGui
)

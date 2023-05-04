find_package(vsg REQUIRED)
find_package(vsgXchange REQUIRED)
find_package(Assimp REQUIRED)

# set the use of C++17 globally as all examples require it
set(CMAKE_CXX_STANDARD 17)

add_executable(Renderer
main.cpp
)

target_compile_definitions(Renderer PRIVATE vsgXchange_FOUND)
target_link_libraries(Renderer
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)


add_executable(VolumeAttempt
volume.cpp
)

target_compile_definitions(VolumeAttempt PRIVATE vsgXchange_FOUND)
target_link_libraries(VolumeAttempt
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)

add_executable(UpdateSceneAttempt
test_updatable_image.cpp
)

target_compile_definitions(UpdateSceneAttempt PRIVATE vsgXchange_FOUND)
target_link_libraries(UpdateSceneAttempt
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)

add_executable(VolumeAttempt2
volume2.cpp
)

target_compile_definitions(VolumeAttempt2 PRIVATE vsgXchange_FOUND)
target_link_libraries(VolumeAttempt2
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
)



add_executable(VolumeAttempt3
volume3.cpp
)

target_compile_definitions(VolumeAttempt3 PRIVATE vsgXchange_FOUND)
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
target_link_libraries(VolumeAttempt5
PUBLIC
${ITK_LIBRARIES}
vsg::vsg
vsgXchange::vsgXchange
)
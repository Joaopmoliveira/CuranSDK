add_executable(test_gaussian_mixture_regression main.cpp)

target_compile_definitions(test_gaussian_mixture_regression PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_gaussian_mixture_regression PUBLIC
utils
gaussianmixture
)


add_executable(test_ransac_filipe test_ransac.cpp)

target_compile_definitions(test_ransac_filipe PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")

target_link_libraries(test_ransac_filipe PUBLIC
utils
gaussianmixture
vsg::vsg
vsgXchange::vsgXchange
${OPERATING_SYSTEM_REQUIRMENTS}
)



add_library(imageprocessing
src/StudyManager.cpp
src/VolumeReconstructor.cpp
src/BoundingBox4Reconstruction.cpp
src/VolumeAlgorithms.cpp
src/FilterAlgorithms.cpp 
src/KernelDescriptor.cpp
src/StaticReconstructor.cpp
src/IntegratedVolumeReconstructor.cpp
src/SplicingTools.cpp
src/igtl2itkConverter.cpp
src/ImageProcessingDefinitions.cpp
src/VolumetricRegistration.cpp
src/PointCloudDownsampling.cpp
src/ArunAlgorithm.cpp
)

find_package(ITK CONFIG REQUIRED)

target_include_directories(imageprocessing
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

include(${ITK_USE_FILE})

target_link_libraries(imageprocessing PUBLIC
${ITK_LIBRARIES}
utils
renderable
userinterface
GeometricTools
OpenIGTLink
${PCL_LIBRARIES}
)

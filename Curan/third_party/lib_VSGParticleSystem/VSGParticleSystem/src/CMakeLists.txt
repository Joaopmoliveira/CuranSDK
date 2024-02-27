set(HEADER_REL_PATH 
    ../include/vsgParticleSystem
)

set(
    AXIS_HEADERS
    ${HEADER_REL_PATH}/axis/_axis.h
    ${HEADER_REL_PATH}/axis/Axis.h
    ${HEADER_REL_PATH}/axis/Text.h
    ${HEADER_REL_PATH}/axis/Arrow.h
    ${HEADER_REL_PATH}/axis/Colorbar.h
)

set(
    BUILDER_HEADERS
    ${HEADER_REL_PATH}/builder/_builder.h
    ${HEADER_REL_PATH}/builder/BuilderProps.h
    ${HEADER_REL_PATH}/builder/ResolutionBuilder.h
)

set(
    DYNAMICS_HEADERS
    ${HEADER_REL_PATH}/dynamics/_dynamics.h
    ${HEADER_REL_PATH}/dynamics/Dynamics.h
    ${HEADER_REL_PATH}/dynamics/GravityField.h
    ${HEADER_REL_PATH}/dynamics/State.h
)

set(
    PARTICLE_HEADERS
    ${HEADER_REL_PATH}/particle/_particle.h
    ${HEADER_REL_PATH}/particle/Particle.h
    ${HEADER_REL_PATH}/particle/ParticleInitializer.h
    ${HEADER_REL_PATH}/particle/ParticlesPool.h
)

set(
    TRANSFORMATION_HEADERS
    ${HEADER_REL_PATH}/transformation/_transformation.h
    ${HEADER_REL_PATH}/transformation/TPipeline.h
    ${HEADER_REL_PATH}/transformation/Transformation.h
    ${HEADER_REL_PATH}/transformation/World2Force.h
    ${HEADER_REL_PATH}/transformation/Normalization.h
    ${HEADER_REL_PATH}/transformation/SoftPlus.h
    ${HEADER_REL_PATH}/transformation/SecurityRegion.h
)

set(
    UTILS_HEADERS
    ${HEADER_REL_PATH}/utils/_utils.h
    ${HEADER_REL_PATH}/utils/vsgAdditionalFunctions.h
    ${HEADER_REL_PATH}/utils/vsgAdditionalOperators.h
    ${HEADER_REL_PATH}/utils/addDemonstrations.h
    ${HEADER_REL_PATH}/utils/stringUtils.h
    ${HEADER_REL_PATH}/utils/additionalFuncs.h
)

set(HEADER_LIST
    ${HEADER_REL_PATH}/all.h
    ${AXIS_HEADERS}
    ${BUILDER_HEADERS}
    ${DYNAMICS_HEADERS}
    ${PARTICLE_HEADERS}
    ${TRANSFORMATION_HEADERS}
    ${UTILS_HEADERS}
)

set(
    AXIS_SOURCE
    axis/Axis.cpp
    axis/Text.cpp
    axis/Arrow.cpp
    axis/Colorbar.cpp
)

set(
    DYNAMICS_SOURCE
    dynamics/Dynamics.cpp
    dynamics/GravityField.cpp
)

set(
    PARTICLE_SOURCE
    particle/Particle.cpp
    particle/ParticleInitializer.cpp
    particle/ParticlesPool.cpp
)

set(
    BUILDER_SOURCE
    builder/ResolutionBuilder.cpp
)

set(
    TRANSFORMATION_SOURCE
    transformation/Transformation.cpp
    transformation/SoftPlus.cpp
    transformation/World2Force.cpp
    transformation/Normalization.cpp
    transformation/TPipeline.cpp
    transformation/SecurityRegion.cpp
)

set(
    UTILS_SOURCE
    utils/addDemonstrations.cpp
    utils/stringUtils.cpp
    utils/vsgAdditionalFunctions.cpp
    utils/additionalFuncs.cpp
)

set(
    SOURCE_LIST
    ${AXIS_SOURCE}
    ${DYNAMICS_SOURCE}
    ${PARTICLE_SOURCE}
    ${BUILDER_SOURCE}
    ${TRANSFORMATION_SOURCE}
    ${UTILS_SOURCE}
)

add_library(vsgps
    SHARED
    ${SOURCE_LIST}
    ${HEADER_LIST}
)

target_include_directories(vsgps PUBLIC ../include)

target_link_libraries(vsgps 
    vsg::vsg
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)


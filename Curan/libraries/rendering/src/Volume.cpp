#include "rendering/Volume.h"

char volume_vert[] = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout(location = 0) in vec3 inPosition;

layout(location = 0) out vec4 cameraPos;
layout(location = 1) out vec4 vertexPos;
layout(location = 2) out mat4 texgen;

out gl_PerVertex{
    vec4 gl_Position;
};

void main() {
    gl_Position = (pc.projection * pc.modelview) * vec4(inPosition, 1.0);
    cameraPos = inverse(pc.modelview) * vec4(0,0,0,1);
    vertexPos = vec4(inPosition, 1.0);
    mat4 temporary = pc.projection * pc.modelview;
    texgen = mat4(1.0);
})";

char volume_frag[] = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(constant_id = 0) const float TransparencyValue = 0.2;
layout(constant_id = 1) const float AlphaFuncValue = 0.1;
layout(constant_id = 2) const float SampleDensityValue = 0.005;

layout(binding = 0) uniform sampler3D volume;

layout(location = 0) in vec4 cameraPos;
layout(location = 1) in vec4 vertexPos;
layout(location = 2) in mat4 texgen;

layout(location = 0) out vec4 outColor;

void main() {
    vec4 t0 = vertexPos;
    vec4 te = cameraPos;
    if( te.x>=0.0 && te.x<=1.0 &&
        te.y>=0.0 && te.y<=1.0 &&
        te.z>=0.0 && te.z<=1.0
        )
    { } else {
        if (te.x<0.0){
            float r = -te.x / (t0.x-te.x);
            te = te + (t0-te)*r;
        }
        if (te.x>1.0){
            float r = (1.0-te.x) / (t0.x-te.x);
            te = te + (t0-te)*r;
        }
        if (te.y<0.0){
            float r = -te.y / (t0.y-te.y);
            te = te + (t0-te)*r;
        }
        if (te.y>1.0){
            float r = (1.0-te.y) / (t0.y-te.y);
            te = te + (t0-te)*r;
        }
        if (te.z<0.0){
            float r = -te.z / (t0.z-te.z);
            te = te + (t0-te)*r;
        }
        if (te.z>1.0){
            float r = (1.0-te.z) / (t0.z-te.z);
            te = te + (t0-te)*r;
        }
    }
    t0 = t0 * texgen;
    te = te * texgen;

    const float min_iteratrions = 2.0;
    const float max_iteratrions = 2048.0;

    float num_iterations = ceil(length((te-t0).xyz)/SampleDensityValue);
    if (num_iterations<min_iteratrions) num_iterations = min_iteratrions;
    else if (num_iterations>max_iteratrions) num_iterations = max_iteratrions;

    vec3 deltaTexCoord=(te-t0).xyz/(num_iterations-1.0);
    vec3 texcoord = t0.xyz;

    vec4 fragColor = vec4(0.0, 0.0, 0.0, 0.0);
    while(num_iterations>0.0)
    {
        float alpha = texture(volume, texcoord).r;
        vec4 color = vec4(alpha, alpha, alpha, alpha * TransparencyValue);
        float r = color.a;
        if (r > AlphaFuncValue)
        {
            fragColor.rgb = mix(fragColor.rgb, color.rgb, r);
            fragColor.a += r;
        }

        if (color.a > fragColor.a)
        {
            fragColor = color;
        }

        texcoord += deltaTexCoord;
        --num_iterations;
    }
    if (fragColor.a>1.0) fragColor.a = 1.0;
    if (fragColor.a<AlphaFuncValue) discard;
    outColor = fragColor;
}
)";

namespace curan {
namespace renderable {

Volume::Volume(Info& info){
    vsg::vec3 origin = vsg::vec3(0.0,0.0,0.0);
    vsg::vec3 dx = vsg::vec3(1.0, 0.0, 0.0);
    vsg::vec3 dy = vsg::vec3(0.0, 1.0, 0.0);
    vsg::vec3 dz = vsg::vec3(0.0, 0.0, 1.0);
    vsg::vec3 v000(origin);
    vsg::vec3 v100(origin + dx);
    vsg::vec3 v110(origin + dx + dy);
    vsg::vec3 v010(origin + dy);
    vsg::vec3 v001(origin + dz);
    vsg::vec3 v101(origin + dx + dz);
    vsg::vec3 v111(origin + dx + dy + dz);
    vsg::vec3 v011(origin + dy + dz);

    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::ushortArray> indices;

    // set up vertex and index arrays
    vertices = vsg::vec3Array::create(
        { v000, v100, v101, v001,   // front
         v100, v110, v111, v101,   // right
         v110, v010, v011, v111,   // far
         v010, v000, v001, v011,   // left
         v010, v110, v100, v000,   // bottom
         v001, v101, v111, v011 }); // top

    indices = vsg::ushortArray::create(
        { 0, 1, 2, 0, 2, 3,
         4, 5, 6, 4, 6, 7,
         8, 9, 10, 8, 10, 11,
         12, 13, 14, 12, 14, 15,
         16, 17, 18, 16, 18, 19,
         20, 21, 22, 20, 22, 23 });

    vsg::DataList arrays;
    arrays.push_back(vertices);

    auto vid = vsg::VertexIndexDraw::create();
    vid->assignArrays(arrays);
    vid->assignIndices(indices);
    vid->indexCount = static_cast<uint32_t>(indices->size());
    uint32_t instanceCount = 1;
    vid->instanceCount = instanceCount;

    float TransparencyValue = 0.3f;
    float AlphaFuncValue = 0.1f;
    float SampleDensityValue = 0.005f;

    float size_x = 1.0;
    float size_y = 1.0;
    float size_z = 1.0;

    vsg::ShaderStage::SpecializationConstants specializationVertexContexts{
        {0, vsg::floatValue::create(size_x)},
        {1, vsg::floatValue::create(size_y)},
        {2, vsg::floatValue::create(size_z)}
    };

    vsg::ShaderStage::SpecializationConstants specializationContexts{
    {0, vsg::floatValue::create(TransparencyValue)},
    {1, vsg::floatValue::create(AlphaFuncValue)},
    {2, vsg::floatValue::create(SampleDensityValue)}
    };

       // load shaders
    auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", volume_vert);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", volume_frag);

    if (!vertexShader || !fragmentShader)
        throw std::runtime_error("failed to create the shaders necessary for the volume rendering");
    
    fragmentShader->specializationConstants = specializationContexts;
    vertexShader->specializationConstants = specializationVertexContexts;

    // read texture image
    textureData = vsg::floatArray3D::create(info.width, info.height, info.depth);
    textureData->properties.format = VK_FORMAT_R32_SFLOAT;
    textureData->properties.dataVariance = vsg::DYNAMIC_DATA;

   // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr} // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128} // projection view, and model matrices, actual push constant calls automatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, // vertex data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, // vertex data
    };

    auto rasterizationState = vsg::RasterizationState::create();
    rasterizationState->cullMode = VK_CULL_MODE_FRONT_BIT;

    vsg::GraphicsPipelineStates pipelineStates{
      vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
      vsg::InputAssemblyState::create(),
      rasterizationState,
      vsg::MultisampleState::create(),
      vsg::ColorBlendState::create(),
      vsg::DepthStencilState::create() };

    auto pipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{ descriptorSetLayout }, pushConstantRanges);
    auto graphicsPipeline = vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{ vertexShader, fragmentShader }, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto clampToEdge_sampler = vsg::Sampler::create();
    clampToEdge_sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    clampToEdge_sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    clampToEdge_sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    clampToEdge_sampler->borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
    clampToEdge_sampler->anisotropyEnable = VK_TRUE;
    clampToEdge_sampler->maxAnisotropy = 1.0;

    auto texture = vsg::DescriptorImage::create(clampToEdge_sampler, textureData, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{ texture });
    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->layout, 0, descriptorSet);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSet);

    vsg::dvec3 position(0.0f, 0.0f, 0.0f);

    double largest_spacing = (info.spacing_x> info.spacing_y) ? info.spacing_x : info.spacing_y;
    largest_spacing = (largest_spacing > info.spacing_z) ? largest_spacing : info.spacing_z;
    vsg::dvec3 scale_spacing(info.spacing_x / largest_spacing, info.spacing_y / largest_spacing, info.spacing_z / largest_spacing);
    

    double largest = (info.width > info.height) ? (double)info.width : (double)info.height;
    largest = (largest> info.depth) ? largest : info.depth;
    vsg::dvec3 scale(info.width/ largest, info.height / largest, info.depth / largest);

    vsg::dvec3 mixture(scale_spacing.x* scale.x, scale_spacing.y* scale.y, scale_spacing.z* scale.z);
    normalize(mixture);

    //auto scalling_transform = vsg::MatrixTransform::create(vsg::scale(info.spacing_x*info.width,info.spacing_y*info.height,info.spacing_z*info.depth));
    auto scalling_transform = vsg::MatrixTransform::create(vsg::scale(mixture));
    transform = vsg::MatrixTransform::create(vsg::translate(position));
    
    // add geometry
    scalling_transform->addChild(vid);
    scenegraph->addChild(scalling_transform);

    obj_contained = vsg::Group::create();
    obj_contained->addChild(scenegraph);

    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Volume::make(Info& info){
    vsg::ref_ptr<Volume> sphere_to_add = Volume::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

void Volume::update_texture(updater&& update){
    update(*(textureData.get()));
    textureData->dirty();
}

}
}
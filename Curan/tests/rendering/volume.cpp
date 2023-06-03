#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>

char inner_vert[] = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragTexCoord;

out gl_PerVertex{
    vec4 gl_Position;
};

void main() {
    gl_Position = (pc.projection * pc.modelview) * vec4(inPosition, 1.0);
    fragColor = inColor;
    fragTexCoord = inTexCoord;
})";


char inner_frag[] = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform sampler3D volume;

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;

layout(location = 0) out vec4 outColor;

void main() {
    float offset = 1.0;
    vec3 indexing = vec3(fragTexCoord.x,fragTexCoord.y,offset);
    float color = texture(volume, indexing).r;
    outColor = vec4(color,color,color,1.0);
})";

void updateBaseTexture3D(vsg::floatArray3D& image, float value)
{
    for (size_t d = 0; d < image.depth(); ++d) {
        float d_ratio = static_cast<float>(d) / static_cast<float>(image.depth() - 1);
        for (size_t r = 0; r < image.height(); ++r)
        {
            float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
            for (size_t c = 0; c < image.width(); ++c)
            {
                float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

                vsg::vec3 delta((r_ratio - 0.5f), (c_ratio - 0.5f),(d_ratio-0.5));

                float distance_from_center = (d_ratio* d_ratio + r_ratio * r_ratio + c_ratio * c_ratio)/sqrt(3.0f);

                float intensity = distance_from_center;
                image.set(c, r, d, intensity);
            }
        }
    }

    image.dirty();
}



vsg::ref_ptr<vsg::Node> get_texture_normal() {
    // load shaders
    auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", inner_vert);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", inner_frag);

    if (!vertexShader || !fragmentShader)
    {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    // read texture image
    auto textureData = vsg::floatArray3D::create(100, 100, 100);
    textureData->properties.format = VK_FORMAT_R32_SFLOAT;
    textureData->properties.dataVariance = vsg::DYNAMIC_DATA;
   
    updateBaseTexture3D(*textureData, 1.0f);
    if (!textureData)
    {
        std::cout << "Could not read texture" << std::endl;
        return {};
    }

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr} // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128} // projection view, and model matrices, actual push constant calls automatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, 32, VK_VERTEX_INPUT_RATE_VERTEX}, // interleaved vertex, color, and texcoord data in a single array
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12}, // colour data
        VkVertexInputAttributeDescription{2, 0, VK_FORMAT_R32G32B32_SFLOAT, 24},    // tex coord data
    };

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::RasterizationState::create(),
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create() };

    auto pipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{ descriptorSetLayout }, pushConstantRanges);
    auto graphicsPipeline = vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{ vertexShader, fragmentShader }, pipelineStates);

    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto sampler = vsg::Sampler::create();
    sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    auto texture = vsg::DescriptorImage::create(sampler, textureData, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{ texture });
    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, descriptorSet);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSet);

    // set up model transformation node
    auto transform = vsg::MatrixTransform::create(); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // set up vertex and index arrays
    auto attributeArray = vsg::floatArray::create(
        { -0.5f, -0.5f, 0.0f,   1.0f, 0.0f, 0.0f,     0.0f, 0.0f  , // x, y,,z,  r, g, b,  s, t, q
           0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f,     1.0f, 0.0f  ,
           0.5f, 0.5f, 0.0f,    0.0f, 0.0f, 1.0f,     1.0f, 1.0f  ,
          -0.5f, 0.5f, 0.0f,    1.0f, 1.0f, 1.0f,     0.0f, 1.0f   });

    auto indices = vsg::ushortArray::create(
        { 0, 1, 2,
          2, 3, 0}); // VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{ attributeArray }));
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(6, 1, 0, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    // add transform to root of the scene graph
    scenegraph->addChild(transform);
    return scenegraph;
};


int main(int argc, char** argv) {
    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->debugLayer = true;
    windowTraits->apiDumpLayer = false;
    windowTraits->width = 1000;
    windowTraits->height = 1000;

    if (arguments.errors()) return arguments.writeErrorMessages(std::cerr);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    auto node = get_texture_normal();
    if (!node)
        throw std::runtime_error("failed to create node");
    scenegraph->addChild(node);

    // create the viewer and assign window(s) to it
    auto viewer = vsg::Viewer::create();

    auto window = vsg::Window::create(windowTraits);
    if (!window)
    {
        std::cout << "Could not create windows." << std::endl;
        return 1;
    }

    viewer->addWindow(window);

    // camera related details
    auto viewport = vsg::ViewportState::create(0, 0, window->extent2D().width, window->extent2D().height);
    auto perspective = vsg::Perspective::create(60.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), 0.1, 10.0);
    auto lookAt = vsg::LookAt::create(vsg::dvec3(1.0, 1.0, 1.0), vsg::dvec3(0.0, 0.0, 0.0), vsg::dvec3(0.0, 0.0, 1.0));
    auto camera = vsg::Camera::create(perspective, lookAt, viewport);


    auto commandGraph = vsg::createCommandGraphForView(window, camera, scenegraph);
    viewer->assignRecordAndSubmitTaskAndPresentation({ commandGraph });

    // compile the Vulkan objects
    viewer->compile();

    // assign a CloseHandler to the Viewer to respond to pressing Escape or press the window close button
    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));

    // main frame loop
    while (viewer->advanceToNextFrame())
    {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }

    // clean up done automatically thanks to ref_ptr<>
    return 0;
}
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>

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
layout(binding = 1) uniform sampler1D transfer_function;

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
        vec4 color = texture( transfer_function, alpha);
        //vec4 color = vec4(alpha, alpha, alpha, alpha * TransparencyValue);
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

#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"

using PixelType = signed short;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

void updateBaseTexture3D(vsg::floatArray3D& image, ImageType::Pointer image_to_render)
{
    using OutputPixelType = float;
    using InputImageType = itk::Image<PixelType, Dimension>;
    using OutputImageType = itk::Image<OutputPixelType, Dimension>;
    using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<OutputImageType, OutputImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try
    {
        rescale->Update();
    }
    catch (const itk::ExceptionObject& e)
    {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    OutputImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<OutputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    {
        ImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
    image.dirty();
}

struct ColorPair {
    double luminance_center;
    double color_variance;
    vsg::vec4 color;
    double scalling_pos_computed;
    double scalling_pre_computed;
};

void create_transfer_function(std::vector<ColorPair>& pairs_of_colors, vsg::vec4Array& transfer_function) {
    for (size_t offset = 0; offset < transfer_function.width(); ++offset) {
        float offset_ratio = static_cast<float>(offset) / static_cast<float>(transfer_function.width() - 1);
        double w = 0.0;
        vsg::vec3 separate_rgb(0.0, 0.0, 0.0);
        std::vector<double> scaling;

        for (const auto& color : pairs_of_colors) {
            float scale = std::exp(-0.5 * ((offset_ratio - color.luminance_center) * (offset_ratio - color.luminance_center)) / (color.color_variance * color.color_variance));
            scaling.push_back(scale);
            w += scale;
        }

        auto iter = pairs_of_colors.begin();
        for (auto& scal : scaling) {
            iter->scalling_pos_computed = scal / w;
            iter->scalling_pre_computed = scal;
            ++iter;
            
        }
        
        for (const auto& color : pairs_of_colors) {
            vsg::vec3 scalled_color(color.color.r* color.scalling_pos_computed, color.color.g * color.scalling_pos_computed, color.color.b * color.scalling_pos_computed);
            //scalled_color = vsg::mix(scalled_color, , (float)iter->scalling_pre_computed);
            separate_rgb += scalled_color;
        }
        if (w > 1.0)
            w = 1.0;
       
        vsg::vec4 composited_color = vsg::vec4(separate_rgb.r, separate_rgb.g, separate_rgb.b,w);
        transfer_function.set(offset, composited_color);
    }
}

vsg::ref_ptr<vsg::Node> get_volume_rendering(vsg::dvec3 position) {
    vsg::vec3 origin = vsg::vec3(position.x, position.y, position.z);
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
    return vid;
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


    // load shaders
    auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", volume_vert);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", volume_frag);

    if (!vertexShader || !fragmentShader)
    {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    float TransparencyValue = 0.3;
    float AlphaFuncValue = 0.1;
    float SampleDensityValue = 0.005;

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

    fragmentShader->specializationConstants = specializationContexts;
    vertexShader->specializationConstants = specializationVertexContexts;

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");
    nameGenerator->SetGlobalWarningDisplay(false);
    std::string dirName{CURAN_COPIED_RESOURCE_PATH"/dicom_sample/mri_brain"};
    nameGenerator->SetDirectory(dirName);

    ImageType::Pointer image_to_render;

    try
    {
        using SeriesIdContainer = std::vector<std::string>;
        const SeriesIdContainer& seriesUID = nameGenerator->GetSeriesUIDs();
        auto                      seriesItr = seriesUID.begin();
        auto                      seriesEnd = seriesUID.end();

        if (seriesItr != seriesEnd)
        {
            //std::cout << "The directory: ";
            //std::cout << dirName << std::endl;
            //std::cout << "Contains the following DICOM Series: ";
            //std::cout << std::endl;
        }
        else
        {
            std::cout << "No DICOMs in: " << dirName << std::endl;
            return EXIT_SUCCESS;
        }

        while (seriesItr != seriesEnd)
        {
            //std::cout << seriesItr->c_str() << std::endl;
            ++seriesItr;
        }

        seriesItr = seriesUID.begin();
        while (seriesItr != seriesUID.end())
        {
            std::string seriesIdentifier;
            if (argc > 3) // If seriesIdentifier given convert only that
            {
                seriesIdentifier = argv[3];
                seriesItr = seriesUID.end();
            }
            else // otherwise convert everything
            {
                seriesIdentifier = seriesItr->c_str();
                seriesItr++;
            }
            //std::cout << "\nReading: ";
            //std::cout << seriesIdentifier << std::endl;
            using FileNamesContainer = std::vector<std::string>;
            FileNamesContainer fileNames = nameGenerator->GetFileNames(seriesIdentifier);

            using ReaderType = itk::ImageSeriesReader<ImageType>;
            auto reader = ReaderType::New();
            using ImageIOType = itk::GDCMImageIO;
            auto dicomIO = ImageIOType::New();
            reader->SetImageIO(dicomIO);
            reader->SetFileNames(fileNames);
            reader->ForceOrthogonalDirectionOff(); // properly read CTs with gantry tilt

            try
            {
                reader->Update();
                image_to_render = reader->GetOutput();
            }
            catch (const itk::ExceptionObject& ex)
            {
                std::cout << ex << std::endl;
                continue;
            }
        }
    }
    catch (const itk::ExceptionObject& ex)
    {
        std::cout << ex << std::endl;
        return EXIT_FAILURE;
    }
    ImageType::RegionType region = image_to_render->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();
    // read texture image
    auto textureData = vsg::floatArray3D::create(size_itk.GetSize()[0], size_itk.GetSize()[1], size_itk.GetSize()[2]);
    textureData->properties.format = VK_FORMAT_R32_SFLOAT;
    textureData->properties.dataVariance = vsg::DYNAMIC_DATA;
    std::cout << "updating texture to render\n";
    updateBaseTexture3D(*textureData, image_to_render);
    std::cout << "updated texture to render\n";
    if (!textureData)
    {
        std::cout << "Could not read texture" << std::endl;
        return {};
    }

    auto transferFunctionData = vsg::vec4Array::create(256);
    transferFunctionData->properties.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    transferFunctionData->properties.dataVariance = vsg::DYNAMIC_DATA;
    std::cout << "updating transfer function to render\n";

    std::vector<ColorPair> color_pairs;
    ColorPair color1;
    color1.color = vsg::vec4(0.89, 0.855, 0.788, 1.0);
    color1.color_variance = 0.01;
    color1.luminance_center = 0.1;
    color_pairs.push_back(color1);

    ColorPair color2;
    color2.color = vsg::vec4(0.9453, 0.6797, 0.6914, 1.0);
    color2.color_variance = 0.005;
    color2.luminance_center = 0.3;
    color_pairs.push_back(color2);
    create_transfer_function(color_pairs,*transferFunctionData);
    std::cout << "updated transfer function to render\n";

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}, // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr} // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
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
    auto texture_transfer_function = vsg::DescriptorImage::create(clampToEdge_sampler, transferFunctionData, 1, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{ texture,texture_transfer_function });
    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->layout, 0, descriptorSet);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSet);

    vsg::dvec3 position(0.0f, 0.0f, 0.0f);

    ImageType::SpacingType spacing = image_to_render->GetSpacing();
    double largest_spacing = (spacing[0] > spacing[1]) ? spacing[0] : spacing[1];
    largest_spacing = (largest_spacing > spacing[2]) ? largest_spacing : spacing[2];
    vsg::dvec3 scale_spacing(spacing[0] / largest_spacing, spacing[1] / largest_spacing, spacing[2] / largest_spacing);

    double largest = (size_itk.GetSize()[0] > size_itk.GetSize()[1]) ? size_itk.GetSize()[0] : size_itk.GetSize()[1];
    largest = (largest > size_itk.GetSize()[2]) ? largest : size_itk.GetSize()[2];
    vsg::dvec3 scale(size_itk.GetSize()[0] / largest, size_itk.GetSize()[1] / largest, size_itk.GetSize()[2] / largest);

    vsg::dvec3 mixture(scale_spacing.x * scale.x, scale_spacing.y * scale.y, scale_spacing.z * scale.z);
    normalize(mixture);

    auto geometry = get_volume_rendering(position);
    auto scalling_transform = vsg::MatrixTransform::create(vsg::scale(mixture));
    auto transform = vsg::MatrixTransform::create(vsg::translate(position));

    // add geometry
    scalling_transform->addChild(geometry);
    transform->addChild(scalling_transform);
    scenegraph->addChild(transform);

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

    vsg::ComputeBounds computeBounds;
    scenegraph->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;
    double nearFarRatio = 0.001;

    // set up the camera
    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 2.0));
    auto viewport = vsg::ViewportState::create(0, 0, window->extent2D().width, window->extent2D().height);
    auto perspective = vsg::Perspective::create(60.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), 0.1, 20.0);
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
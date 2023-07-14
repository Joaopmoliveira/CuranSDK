#include "rendering/integrated_shaders/VolumeIntegratedShaders.h"
#include "imageprocessing/TemplatedVolumeAlgorithms.h"
#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "itkImportImageFilter.h"

namespace curan{
namespace image {

IntegratedReconstructor::Info::Info(std::array<double,3> inspacing,std::array<double,3> inorigin, std::array<double,3> insize, std::array<std::array<double,3>,3> indirection){
	auto inextent = gte::Vector3<double>{insize[0]/2.0,insize[1]/2.0,insize[2]/2.0};

    auto origin = gte::Vector3<double>{inorigin[0],inorigin[1],inorigin[2]};
	std::array<gte::Vector3<double>, 3> alignement;
	alignement[0] = {indirection[0][0],indirection[0][1],indirection[0][2]};
	alignement[1] = {indirection[1][0],indirection[1][1],indirection[1][2]};
	alignement[2] = {indirection[2][0],indirection[2][1],indirection[2][2]};

    auto output_origin = origin
		+ alignement[0] * inextent[0]
		+ alignement[1] * inextent[1]
		+ alignement[2] * inextent[2];

    volumetric_bounding_box = gte::OrientedBox3<double>{output_origin,alignement,inextent};
    spacing[0] = inspacing[0];
    spacing[1] = inspacing[1];
    spacing[2] = inspacing[2];
}

vsg::ref_ptr<renderable::Renderable> IntegratedReconstructor::make(Info& info){
    vsg::ref_ptr<IntegratedReconstructor> sphere_to_add = IntegratedReconstructor::create(info);
    vsg::ref_ptr<renderable::Renderable> val = sphere_to_add.cast<renderable::Renderable>();
    return val;
}

IntegratedReconstructor::IntegratedReconstructor(const Info& info){
	output_type::IndexType output_start;
	output_start[0] = 0;
    output_start[1] = 0;
    output_start[2] = 0;

    output_type::DirectionType output_directorion;
    output_directorion[0][0] = volumetric_bounding_box.axis[0][0];
    output_directorion[1][0] = volumetric_bounding_box.axis[1][0];
    output_directorion[2][0] = volumetric_bounding_box.axis[2][0];

    output_directorion[0][1] = volumetric_bounding_box.axis[0][1];
    output_directorion[1][1] = volumetric_bounding_box.axis[1][1];
    output_directorion[2][1] = volumetric_bounding_box.axis[2][1];

    output_directorion[0][2] = volumetric_bounding_box.axis[0][2];
    output_directorion[1][2] = volumetric_bounding_box.axis[1][2];
    output_directorion[2][2] = volumetric_bounding_box.axis[2][2];

	output_type::SizeType output_size;
    output_size[0] = std::ceil(volumetric_bounding_box.extent[0] * 2 / output_spacing[0]);
    output_size[1] = std::ceil(volumetric_bounding_box.extent[1] * 2 / output_spacing[1]);
    output_size[2] = std::ceil(volumetric_bounding_box.extent[2] * 2 / output_spacing[2]);

	gte::Vector<3, double> origin_gte = volumetric_bounding_box.center
		- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
		- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
		- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

    output_type::PointType output_origin;
    output_origin[0] = origin_gte[0];
    output_origin[1] = origin_gte[1];	
    output_origin[2] = origin_gte[2];
    

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

    float TransparencyValue = 0.5f;
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
    auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", renderable::volume_vert);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", renderable::volume_frag);

    if (!vertexShader || !fragmentShader)
        throw std::runtime_error("failed to create the shaders necessary for the volume rendering");
    
    fragmentShader->specializationConstants = specializationContexts;
    vertexShader->specializationConstants = specializationVertexContexts;

    // read texture image
    textureData = vsg::floatArray3D::create(output_size[0], output_size[1], output_size[2]);
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

    vsg::dvec3 mixture(output_size[0]* info.spacing[0] * 0.001,output_size[1]* info.spacing[1]* 0.001, output_size[2]* info.spacing[2]* 0.001);

    auto scalling_transform = vsg::MatrixTransform::create(vsg::scale(mixture));
    transform = vsg::MatrixTransform::create(vsg::translate(position));
    
    // add geometry
    scenegraph->addChild(vid);
    scalling_transform->addChild(scenegraph);

    obj_contained = vsg::Group::create();
    obj_contained->addChild(scalling_transform);

    output_type::RegionType output_region;
    output_region.SetSize(output_size);
    output_region.SetIndex(output_start);

    using ImportFilterType = itk::ImportImageFilter<output_pixel_type, Dimension>;

    auto importFilter = ImportFilterType::New();
    importFilter->SetRegion(output_region);
    importFilter->SetOrigin(output_origin);
    importFilter->SetSpacing(output_spacing);
    importFilter->SetDirection(output_directorion);
    const bool importImageFilterWillOwnTheBuffer = false;
    importFilter->SetImportPointer(textureData->data(), textureData->size(), importImageFilterWillOwnTheBuffer);    
    importFilter-update();
    out_volume = importFilter->GetOutput();

	acummulation_buffer = accumulator_type::New();
	acummulation_buffer->SetRegions(output_region);
	acummulation_buffer->SetOrigin(output_origin);
	acummulation_buffer->SetSpacing(output_spacing);
	acummulation_buffer->SetDirection(output_directorion);
	acummulation_buffer->Allocate(true);

    if (info.identifier)
        set_identifier(*info.identifier);
}

IntegratedReconstructor::~IntegratedReconstructor(){

}

IntegratedReconstructor& IntegratedReconstructor::set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy){
	std::lock_guard<std::mutex> g{mut};
    interpolation_strategy = new_interpolation_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy){
	std::lock_guard<std::mutex> g{mut};
    compounding_strategy = new_compounding_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_fillstrategy(const curan::image::reconstruction::Compounding& new_compounding_strategy){
	std::lock_guard<std::mutex> g{mut};
    compounding_strategy = new_compounding_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_clipping(const Clipping& new_clipping){
	std::lock_guard<std::mutex> g{mut};
    clipping = new_clipping;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::add_frame(input_type::Pointer image_pointer){
	std::lock_guard<std::mutex> g{mut};
	frame_data.push_back(image_pointer);
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::add_frames(std::vector<input_type::Pointer>& images_vector){
	std::lock_guard<std::mutex> g{mut};
    frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
    return *(this);
}

IntegratedReconstructor::output_type::Pointer IntegratedReconstructor::get_output_pointer(){
    return out_volume;
}

bool IntegratedReconstructor::update(){
	gte::Vector<3, double> output_origin = volumetric_bounding_box.center
	- volumetric_bounding_box.axis[0] * volumetric_bounding_box.extent[0]
	- volumetric_bounding_box.axis[1] * volumetric_bounding_box.extent[1]
	- volumetric_bounding_box.axis[2] * volumetric_bounding_box.extent[2];

	Eigen::Matrix4d ref_to_output_origin;
	ref_to_output_origin(0, 0) = volumetric_bounding_box.axis[0][0];
	ref_to_output_origin(1, 0) = volumetric_bounding_box.axis[0][1];
	ref_to_output_origin(2, 0) = volumetric_bounding_box.axis[0][2];
	ref_to_output_origin(3, 0) = 0.0;

	ref_to_output_origin(0, 1) = volumetric_bounding_box.axis[1][0];
	ref_to_output_origin(1, 1) = volumetric_bounding_box.axis[1][1];
	ref_to_output_origin(2, 1) = volumetric_bounding_box.axis[1][2];
	ref_to_output_origin(3, 1) = 0.0;

	ref_to_output_origin(0, 2) = volumetric_bounding_box.axis[2][0];
	ref_to_output_origin(1, 2) = volumetric_bounding_box.axis[2][1];
	ref_to_output_origin(2, 2) = volumetric_bounding_box.axis[2][2];
	ref_to_output_origin(3, 2) = 0.0;

	ref_to_output_origin(0, 3) = output_origin[0];
	ref_to_output_origin(1, 3) = output_origin[1];
	ref_to_output_origin(2, 3) = output_origin[2];
	ref_to_output_origin(3, 3) = 1.0;

	Eigen::Matrix4d output_to_ref = ref_to_output_origin.inverse();

	unsigned int accOverflow = 20;

	curan::image::reconstruction::PasteSliceIntoVolumeInsertSliceParamsTemplated<input_pixel_type,output_pixel_type> paste_slice_info;
	paste_slice_info.outPtr = out_volume->GetBufferPointer();
    auto size_out = out_volume->GetLargestPossibleRegion().GetSize();
    auto origin_out = out_volume->GetOrigin();
    auto spacing_out = out_volume->GetSpacing();
    paste_slice_info.out_origin[0] = origin_out[0];
    paste_slice_info.out_origin[1] = origin_out[1];
    paste_slice_info.out_origin[2] = origin_out[2];
    paste_slice_info.out_size[0] = size_out[0];
    paste_slice_info.out_size[1] = size_out[1];
    paste_slice_info.out_size[2] = size_out[2];
    paste_slice_info.out_spacing[0] = spacing_out[0];
    paste_slice_info.out_spacing[1] = spacing_out[1];
    paste_slice_info.out_spacing[2] = spacing_out[2];
	paste_slice_info.accPtr = acummulation_buffer->GetBufferPointer();
	paste_slice_info.interpolationMode = interpolation_strategy;
	paste_slice_info.compoundingMode = compounding_strategy;
	paste_slice_info.accOverflowCount = &accOverflow;
	paste_slice_info.pixelRejectionThreshold = 0;
	paste_slice_info.image_number = 0;

	Eigen::Matrix4d ref_to_image;
	ref_to_image(3, 0) = 0.0;
	ref_to_image(3, 1) = 0.0;
	ref_to_image(3, 2) = 0.0;
	ref_to_image(3, 3) = 1.0;

	// cicle throught all frames and insert
	// them in the output buffer, one at a time

	std::vector<input_type::Pointer> local_image_copies;
	{
		std::lock_guard<std::mutex> g{mut};
		local_image_copies = std::move(frame_data);
		frame_data = std::vector<input_type::Pointer>();
		if(local_image_copies.size()==0)
			return false;
	}

	for (auto img : local_image_copies) {	
	    int inputFrameExtentForCurrentThread[6] = { 0, 0, 0, 0, 0, 0 };
		double clipRectangleOrigin [2]; // array size 2
		double clipRectangleSize [2]; // array size 2
		if(clipping){
			clipRectangleOrigin[0] = (*clipping).clipRectangleOrigin[0];
			clipRectangleOrigin[1] = (*clipping).clipRectangleOrigin[1];

			clipRectangleSize[0] = (*clipping).clipRectangleSize[0];
			clipRectangleSize[1] = (*clipping).clipRectangleSize[1];

			inputFrameExtentForCurrentThread[1] = clipRectangleSize[0];
			inputFrameExtentForCurrentThread[3] = clipRectangleSize[1];
		} else {
			auto local_size = img->GetLargestPossibleRegion().GetSize();
			auto local_origin = img->GetOrigin();
			clipRectangleOrigin[0] = local_origin[0];
			clipRectangleOrigin[1] = local_origin[1];
			clipRectangleSize[0] = local_size.GetSize()[0]-1;
			clipRectangleSize[1] = local_size.GetSize()[1]-1;

			inputFrameExtentForCurrentThread[1] = clipRectangleSize[0];
			inputFrameExtentForCurrentThread[3] = clipRectangleSize[1];
		}	
		paste_slice_info.clipRectangleOrigin = clipRectangleOrigin;
	    paste_slice_info.clipRectangleSize = clipRectangleSize;
		paste_slice_info.inExt = inputFrameExtentForCurrentThread;
		paste_slice_info.image_number += 1;

		itk::Matrix<double> image_orientation = img->GetDirection();
		itk::Point<double> image_origin = img->GetOrigin();

		ref_to_image(0, 0) = image_orientation[0][0];
		ref_to_image(1, 0) = image_orientation[1][0];
		ref_to_image(2, 0) = image_orientation[2][0];

		ref_to_image(0, 1) = image_orientation[0][1];
		ref_to_image(1, 1) = image_orientation[1][1];
		ref_to_image(2, 1) = image_orientation[2][1];

		ref_to_image(0, 2) = image_orientation[0][2];
		ref_to_image(1, 2) = image_orientation[1][2];
		ref_to_image(2, 2) = image_orientation[2][2];

		ref_to_image(0, 3) = image_origin[0];
		ref_to_image(1, 3) = image_origin[1];
		ref_to_image(2, 3) = image_origin[2];

		// The matrix is the transformation of the 
		// origin of the output volume (1) to the 
		// origin of the input image (2). This is 
		// given by T02=T01*T12, and by premultiplying 
		// by T10=inverse(T01) we obtain T12=inverse(T01)*T02
		Eigen::Matrix4d output_to_origin = output_to_ref * ref_to_image;
        auto size_in = img->GetLargestPossibleRegion().GetSize();
        auto origin_in = img->GetOrigin();
        auto spacing_in = img->GetSpacing();
		paste_slice_info.inPtr = img->GetBufferPointer();
        paste_slice_info.in_origin[0] = origin_in[0];
        paste_slice_info.in_origin[1] = origin_in[1];
        paste_slice_info.in_origin[2] = origin_in[2];
        paste_slice_info.in_size[0] = size_in[0];
        paste_slice_info.in_size[1] = size_in[1];
        paste_slice_info.in_size[2] = size_in[2];
        paste_slice_info.in_spacing[0] = spacing_in[0];
        paste_slice_info.in_spacing[1] = spacing_in[1];
        paste_slice_info.in_spacing[2] = spacing_in[2];
		paste_slice_info.matrix = output_to_origin;
		curan::image::reconstruction::TemplatedUnoptimizedInsertSlice<input_pixel_type,output_pixel_type,255>(&paste_slice_info);
	};
    textureData->dirty();
    return true;
}

bool IntegratedReconstructor::multithreaded_update(std::shared_ptr<utilities::ThreadPool>pool){
    return true;
}

}
}